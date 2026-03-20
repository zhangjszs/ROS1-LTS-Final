#!/usr/bin/env python3
"""
Evaluate color semantics consistency from replay result bags.

Rule:
  - RIGHT boundary: BLUE and y < 0
  - LEFT boundary: RED/YELLOW_SMALL/YELLOW_BIG and y > 0
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional


COLOR_NAMES = {
    0: "blue",
    1: "yellow_small",
    2: "yellow_big",
    3: "red",
    4: "none",
}


def _ratio(ok: int, total: int) -> Optional[float]:
    if total <= 0:
        return None
    return ok / float(total)


def _resolve_color_field(msg: Any) -> str:
    if hasattr(msg, "fused_color_types"):
        return "fused_color_types"
    if hasattr(msg, "color_types"):
        return "color_types"
    raise AttributeError("message has neither fused_color_types nor color_types")


def evaluate_bag(bag_path: Path, topic: str) -> Dict[str, Any]:
    try:
        import rosbag
    except Exception as exc:
        raise RuntimeError("rosbag Python package is required (source ROS environment).") from exc

    frames = 0
    det_total = 0
    counts = {name: 0 for name in COLOR_NAMES.values()}
    blue_ok = 0
    blue_total = 0
    left_ok = 0
    left_total = 0

    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            frames += 1
            color_field = _resolve_color_field(msg)
            colors = list(getattr(msg, color_field, []))
            points = list(getattr(msg, "points", []))
            n = min(len(colors), len(points))
            det_total += n
            for i in range(n):
                c = int(colors[i])
                y = float(points[i].y)
                counts[COLOR_NAMES.get(c, "none")] += 1
                if c == 0:
                    blue_total += 1
                    if y < 0.0:
                        blue_ok += 1
                elif c in (1, 2, 3):
                    left_total += 1
                    if y > 0.0:
                        left_ok += 1

    overall_ok = blue_ok + left_ok
    overall_total = blue_total + left_total
    none_count = counts["none"]
    none_rate = (none_count / float(det_total)) if det_total > 0 else None

    return {
        "bag": str(bag_path),
        "topic": topic,
        "frames": frames,
        "detections_total": det_total,
        "counts": counts,
        "blue_right_consistency": {
            "ok": blue_ok,
            "total": blue_total,
            "rate": _ratio(blue_ok, blue_total),
        },
        "left_boundary_consistency": {
            "ok": left_ok,
            "total": left_total,
            "rate": _ratio(left_ok, left_total),
        },
        "overall_boundary_consistency": {
            "ok": overall_ok,
            "total": overall_total,
            "rate": _ratio(overall_ok, overall_total),
        },
        "none_rate": none_rate,
    }


def _fmt(x: Optional[float], ndigits: int = 4) -> str:
    if x is None:
        return "NA"
    return f"{x:.{ndigits}f}"


def _check_thresholds(metrics: Dict[str, Any], args: argparse.Namespace) -> List[str]:
    violations: List[str] = []
    frames = int(metrics["frames"])
    blue_rate = metrics["blue_right_consistency"]["rate"]
    left_rate = metrics["left_boundary_consistency"]["rate"]
    overall_rate = metrics["overall_boundary_consistency"]["rate"]
    none_rate = metrics["none_rate"]

    if args.min_frames is not None and frames < args.min_frames:
        violations.append(f"frames={frames} < min_frames={args.min_frames}")
    if args.min_blue_right is not None:
        if blue_rate is None or blue_rate < args.min_blue_right:
            violations.append(
                f"blue_right={_fmt(blue_rate)} < min_blue_right={args.min_blue_right:.4f}"
            )
    if args.min_left_boundary is not None:
        if left_rate is None or left_rate < args.min_left_boundary:
            violations.append(
                f"left_boundary={_fmt(left_rate)} < min_left_boundary={args.min_left_boundary:.4f}"
            )
    if args.min_overall is not None:
        if overall_rate is None or overall_rate < args.min_overall:
            violations.append(f"overall={_fmt(overall_rate)} < min_overall={args.min_overall:.4f}")
    if args.max_none_rate is not None:
        if none_rate is None or none_rate > args.max_none_rate:
            violations.append(f"none_rate={_fmt(none_rate)} > max_none_rate={args.max_none_rate:.4f}")

    return violations


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate color semantics consistency from fused detection replay bags."
    )
    parser.add_argument("bag_file", type=Path, help="Input replay result bag path.")
    parser.add_argument(
        "--topic",
        default="/perception/fusion/detections",
        help="Detection topic (default: /perception/fusion/detections)",
    )
    parser.add_argument("-o", "--output-json", type=Path, default=None, help="Output JSON path.")
    parser.add_argument("--min-frames", type=int, default=None, help="Minimum required frame count.")
    parser.add_argument(
        "--min-blue-right",
        type=float,
        default=None,
        help="Minimum BLUE->right consistency rate.",
    )
    parser.add_argument(
        "--min-left-boundary",
        type=float,
        default=None,
        help="Minimum LEFT-boundary->left consistency rate.",
    )
    parser.add_argument(
        "--min-overall",
        type=float,
        default=None,
        help="Minimum overall boundary consistency rate.",
    )
    parser.add_argument(
        "--max-none-rate",
        type=float,
        default=None,
        help="Maximum allowed NONE color rate.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not args.bag_file.exists():
        print(f"ERROR: bag file not found: {args.bag_file}", file=sys.stderr)
        return 2

    metrics = evaluate_bag(args.bag_file, args.topic)

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n")

    print("Color Semantics Metrics")
    print(f"  bag:            {metrics['bag']}")
    print(f"  topic:          {metrics['topic']}")
    print(f"  frames:         {metrics['frames']}")
    print(f"  detections:     {metrics['detections_total']}")
    print(f"  blue->right:    {_fmt(metrics['blue_right_consistency']['rate'])}")
    print(f"  left->left:     {_fmt(metrics['left_boundary_consistency']['rate'])}")
    print(f"  overall:        {_fmt(metrics['overall_boundary_consistency']['rate'])}")
    print(f"  none_rate:      {_fmt(metrics['none_rate'])}")

    violations = _check_thresholds(metrics, args)
    if violations:
        print("RESULT: FAILED")
        for item in violations:
            print(f"  - {item}")
        return 1

    print("RESULT: PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
