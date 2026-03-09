#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ConeMap evaluation — confidence distribution proxies from bag replay.

Usage:
  python3 evaluate_cone_map_metrics.py <bag_file> [-o output.json]
      [--topic /localization/cone_map]
      [--dist-th 18.0]
      [--y-th 1.0]

Outputs:
  - message/cone counts
  - confidence quantiles and histograms for:
      * all cones
      * dist >= dist_th
      * dist >= dist_th and abs(y) <= y_th
      * dist >= dist_th and abs(y) >  y_th
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

DEFAULT_TOPIC = "/localization/cone_map"


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _confidence_score(confidence_scaled: int) -> float:
    return _clamp(float(confidence_scaled) / 1000.0, 0.0, 1.0)


def _quantiles(
    values: List[float], qs=(0.10, 0.50, 0.90, 0.95, 0.99)
) -> Dict[str, Optional[float]]:
    if not values:
        return {f"p{int(q * 100):02d}": None for q in qs}
    s = sorted(values)
    n = len(s)
    out: Dict[str, Optional[float]] = {}
    for q in qs:
        idx = int(round(q * (n - 1)))
        out[f"p{int(q * 100):02d}"] = round(s[idx], 4)
    return out


def _histogram(values: List[float], bin_width: float = 0.05) -> Dict[str, int]:
    if bin_width <= 0:
        bin_width = 0.05
    bins: Dict[str, int] = {}
    for v in values:
        vv = _clamp(v, 0.0, 1.0)
        start = math.floor(vv / bin_width) * bin_width
        if start >= 1.0:
            start = 1.0 - bin_width
        end = min(1.0, start + bin_width)
        key = f"{start:.2f}-{end:.2f}"
        bins[key] = bins.get(key, 0) + 1
    return dict(sorted(bins.items(), key=lambda kv: float(kv[0].split("-")[0])))


def _fraction_at_or_above(
    values: List[float],
    thresholds=(0.2, 0.4, 0.6, 0.75, 0.8, 0.83, 0.84, 0.85, 0.86, 0.88, 0.9, 0.92, 0.95),
) -> Dict[str, float]:
    if not values:
        return {f">={t:.2f}": 0.0 for t in thresholds}
    n = len(values)
    out: Dict[str, float] = {}
    for t in thresholds:
        out[f">={t:.2f}"] = round(sum(1 for v in values if v >= t) / n, 4)
    return out


def _load_confidences_from_bag(
    bag_path: Path,
    topic: str,
    dist_th: float,
    y_th: Optional[float],
    t0: Optional[float] = None,
    t1: Optional[float] = None,
) -> Dict[str, Any]:
    try:
        import rosbag
    except ImportError:
        print("ERROR: rosbag not installed. Install with: pip install rosbag", file=sys.stderr)
        sys.exit(1)

    total_msgs = 0
    total_cones = 0

    all_scores: List[float] = []
    far_scores: List[float] = []
    far_center_scores: List[float] = []
    far_side_scores: List[float] = []
    front_far_scores: List[float] = []
    front_far_center_scores: List[float] = []
    front_far_side_scores: List[float] = []

    dist_th = float(dist_th)
    if y_th is not None:
        y_th = float(y_th)

    with rosbag.Bag(str(bag_path), "r") as bag:
        for _topic_name, msg, _t in bag.read_messages(topics=[topic]):
            stamp_s = msg.header.stamp.to_sec() if hasattr(msg, "header") else _t.to_sec()
            if t0 is not None and stamp_s < t0:
                continue
            if t1 is not None and stamp_s > t1:
                continue
            total_msgs += 1
            cones = getattr(msg, "cone", [])
            for c in cones:
                total_cones += 1
                score = _confidence_score(int(getattr(c, "confidence", 0)))
                all_scores.append(score)

                pos = getattr(c, "position_baseLink", None)
                if pos is None:
                    continue
                dist = math.hypot(float(getattr(pos, "x", 0.0)), float(getattr(pos, "y", 0.0)))
                if dist < dist_th:
                    continue
                far_scores.append(score)
                x = float(getattr(pos, "x", 0.0))
                y = float(getattr(pos, "y", 0.0))
                if x >= 0.0:
                    front_far_scores.append(score)
                if y_th is None:
                    continue
                if abs(y) <= y_th:
                    far_center_scores.append(score)
                    if x >= 0.0:
                        front_far_center_scores.append(score)
                else:
                    far_side_scores.append(score)
                    if x >= 0.0:
                        front_far_side_scores.append(score)

    return {
        "bag_file": str(bag_path),
        "topic": topic,
        "dist_th": dist_th,
        "y_th": y_th,
        "counts": {
            "messages": total_msgs,
            "cones_total": total_cones,
            "cones_dist_ge_th": len(far_scores),
            "cones_dist_ge_th_and_abs_y_le_th": len(far_center_scores)
            if y_th is not None
            else None,
            "cones_dist_ge_th_and_abs_y_gt_th": len(far_side_scores) if y_th is not None else None,
            "cones_x_ge_0_and_dist_ge_th": len(front_far_scores),
            "cones_x_ge_0_and_dist_ge_th_and_abs_y_le_th": len(front_far_center_scores)
            if y_th is not None
            else None,
            "cones_x_ge_0_and_dist_ge_th_and_abs_y_gt_th": len(front_far_side_scores)
            if y_th is not None
            else None,
        },
        "confidence": {
            "all": {
                "quantiles": _quantiles(all_scores),
                "fraction_at_or_above": _fraction_at_or_above(all_scores),
                "hist": _histogram(all_scores),
            },
            "dist_ge_th": {
                "quantiles": _quantiles(far_scores),
                "fraction_at_or_above": _fraction_at_or_above(far_scores),
                "hist": _histogram(far_scores),
            },
            "dist_ge_th_and_abs_y_le_th": None
            if y_th is None
            else {
                "quantiles": _quantiles(far_center_scores),
                "fraction_at_or_above": _fraction_at_or_above(far_center_scores),
                "hist": _histogram(far_center_scores),
            },
            "dist_ge_th_and_abs_y_gt_th": None
            if y_th is None
            else {
                "quantiles": _quantiles(far_side_scores),
                "fraction_at_or_above": _fraction_at_or_above(far_side_scores),
                "hist": _histogram(far_side_scores),
            },
            "x_ge_0_and_dist_ge_th": {
                "quantiles": _quantiles(front_far_scores),
                "fraction_at_or_above": _fraction_at_or_above(front_far_scores),
                "hist": _histogram(front_far_scores),
            },
            "x_ge_0_and_dist_ge_th_and_abs_y_le_th": None
            if y_th is None
            else {
                "quantiles": _quantiles(front_far_center_scores),
                "fraction_at_or_above": _fraction_at_or_above(front_far_center_scores),
                "hist": _histogram(front_far_center_scores),
            },
            "x_ge_0_and_dist_ge_th_and_abs_y_gt_th": None
            if y_th is None
            else {
                "quantiles": _quantiles(front_far_side_scores),
                "fraction_at_or_above": _fraction_at_or_above(front_far_side_scores),
                "hist": _histogram(front_far_side_scores),
            },
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Evaluate ConeMap confidence distribution from rosbag."
    )
    parser.add_argument("bag", help="Path to rosbag file")
    parser.add_argument(
        "-t", "--topic", default=DEFAULT_TOPIC, help=f"Topic (default: {DEFAULT_TOPIC})"
    )
    parser.add_argument("-o", "--output", default=None, help="Output JSON file (default: stdout)")
    parser.add_argument(
        "--dist-th", type=float, default=18.0, help="Distance threshold in base_link [m]"
    )
    parser.add_argument("--y-th", type=float, default=1.0, help="|y| threshold in base_link [m]")
    parser.add_argument(
        "--t0", type=float, default=None, help="Start time [s] (filter by message stamp, inclusive)"
    )
    parser.add_argument(
        "--t1", type=float, default=None, help="End time [s] (filter by message stamp, inclusive)"
    )
    args = parser.parse_args()

    bag_path = Path(args.bag)
    if not bag_path.exists():
        print(f"ERROR: bag not found: {bag_path}", file=sys.stderr)
        sys.exit(2)

    result = _load_confidences_from_bag(
        bag_path,
        args.topic,
        args.dist_th,
        args.y_th,
        t0=args.t0,
        t1=args.t1,
    )

    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
    else:
        json.dump(result, sys.stdout, ensure_ascii=False, indent=2)
        print()


if __name__ == "__main__":
    main()
