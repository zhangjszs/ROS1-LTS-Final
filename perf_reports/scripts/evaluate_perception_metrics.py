#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Perception LiDAR evaluation — proxy metrics from bag replay.

Usage:
    # Proxy metrics only (no ground truth):
    python3 evaluate_perception_metrics.py <bag_file> [-o output.json]

    # With ground-truth cone map (adds precision/recall/F1/RMSE):
    python3 evaluate_perception_metrics.py <bag_file> --gt cones_gt.csv

Proxy metrics (always computed, no GT required):
  1. n_frames              — total frames processed
  2. mean_detections       — average detections per frame
  3. std_detections        — std dev of detections per frame (stability)
  4. spike_rate            — fraction of frames with |Δn| > 2*std (sudden jumps)
  5. zero_frame_rate       — fraction of frames with 0 detections
  6. mean_confidence       — average confidence across all detections
  7. std_confidence        — std dev of confidence
  8. mean_distance_m       — average detection distance [m]
  9. symmetry_ratio        — left/right detection balance (1.0 = perfect)
 10. mean_frame_interval_s — average time between frames [s]

GT metrics (when --gt is provided):
 11. precision             — TP / (TP + FP)
 12. recall                — TP / (TP + FN)
 13. f1                    — harmonic mean of precision and recall
 14. rmse_m                — RMSE of matched detection positions [m]
 15. mean_gt_visible       — average GT cones within sensor range per frame
 16. gt_match_threshold_m  — matching distance threshold used [m]

GT file format (CSV, one cone per line):
    x,y,z
    1.23,4.56,0.0
    ...
  Coordinates must be in the same frame as detections (typically velodyne).
  For static GT maps, the bag should be recorded with the vehicle stationary
  or the GT must be transformed to ego-frame per-frame (see --gt-frame).
"""

import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

DEFAULT_TOPIC = "/perception/lidar_cluster/detections"


def _safe_mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _safe_std(values: List[float], mean: float) -> float:
    if len(values) < 2:
        return 0.0
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def _load_gt_cones(gt_path: Path) -> List[Tuple[float, float, float]]:
    """Load ground-truth cone positions from CSV (x,y,z per line)."""
    cones: List[Tuple[float, float, float]] = []
    with open(gt_path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            # Skip header row
            try:
                x, y, z = float(row[0]), float(row[1]), float(row[2]) if len(row) > 2 else 0.0
            except ValueError:
                continue  # skip header or malformed lines
            cones.append((x, y, z))
    return cones


def _greedy_match(
    detections: List[Tuple[float, float]],
    gt_cones: List[Tuple[float, float]],
    threshold: float,
) -> Tuple[int, int, int, List[float]]:
    """Greedy nearest-neighbor matching (O(n*m), sufficient for <500 cones).

    Returns (tp, fp, fn, matched_distances).
    """
    if not gt_cones:
        return 0, len(detections), 0, []
    if not detections:
        return 0, 0, len(gt_cones), []

    gt_matched = [False] * len(gt_cones)
    matched_dists: List[float] = []
    tp = 0

    for dx, dy in detections:
        best_j = -1
        best_d = threshold
        for j, (gx, gy) in enumerate(gt_cones):
            if gt_matched[j]:
                continue
            d = math.sqrt((dx - gx) ** 2 + (dy - gy) ** 2)
            if d < best_d:
                best_d = d
                best_j = j
        if best_j >= 0:
            gt_matched[best_j] = True
            tp += 1
            matched_dists.append(best_d)

    fp = len(detections) - tp
    fn = sum(1 for m in gt_matched if not m)
    return tp, fp, fn, matched_dists


def _evaluate_gt(
    frames: List[Dict[str, Any]],
    gt_cones: List[Tuple[float, float, float]],
    match_threshold: float,
    max_range: float,
) -> Dict[str, Any]:
    """Compute GT-based metrics across all frames."""
    gt_xy = [(x, y) for x, y, _ in gt_cones]

    # Filter GT to sensor range
    gt_in_range = [(x, y) for x, y in gt_xy if math.sqrt(x * x + y * y) <= max_range]

    total_tp = 0
    total_fp = 0
    total_fn = 0
    all_matched_dists: List[float] = []

    for frame in frames:
        det_xy = list(zip(frame["det_x"], frame["det_y"]))
        tp, fp, fn, dists = _greedy_match(det_xy, gt_in_range, match_threshold)
        total_tp += tp
        total_fp += fp
        total_fn += fn
        all_matched_dists.extend(dists)

    precision = total_tp / max(total_tp + total_fp, 1)
    recall = total_tp / max(total_tp + total_fn, 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-9)
    rmse = math.sqrt(_safe_mean([d * d for d in all_matched_dists])) if all_matched_dists else 0.0

    return {
        "precision": round(precision, 4),
        "recall": round(recall, 4),
        "f1": round(f1, 4),
        "rmse_m": round(rmse, 4),
        "total_tp": total_tp,
        "total_fp": total_fp,
        "total_fn": total_fn,
        "mean_gt_visible": len(gt_in_range),
        "gt_match_threshold_m": match_threshold,
        "gt_max_range_m": max_range,
    }


def _extract_frames(bag_path: Path, topic: str) -> List[Dict[str, Any]]:
    """Read bag and extract per-frame data from ConeDetections messages."""
    try:
        import rosbag
    except ImportError:
        print("ERROR: rosbag Python package not found.", file=sys.stderr)
        print("  Install via: pip install rosbag  (or source your ROS workspace)", file=sys.stderr)
        sys.exit(1)

    frames: List[Dict[str, Any]] = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _topic, msg, t in bag.read_messages(topics=[topic]):
            n_det = len(msg.points)
            confidences = [float(c) for c in msg.confidence] if msg.confidence else []
            distances = [float(d) for d in msg.obj_dist] if msg.obj_dist else []

            # Per-detection positions (for GT matching)
            det_x = [float(p.x) for p in msg.points]
            det_y = [float(p.y) for p in msg.points]

            # Count left (y > 0) vs right (y <= 0) detections
            n_left = sum(1 for p in msg.points if p.y > 0)
            n_right = n_det - n_left

            frames.append({
                "stamp_sec": msg.header.stamp.to_sec(),
                "n_detections": n_det,
                "confidences": confidences,
                "distances": distances,
                "det_x": det_x,
                "det_y": det_y,
                "n_left": n_left,
                "n_right": n_right,
            })
    return frames

def evaluate(
    bag_path: Path,
    topic: str,
    gt_path: Optional[Path] = None,
    gt_match_threshold: float = 1.0,
    gt_max_range: float = 50.0,
) -> Dict[str, Any]:
    """Compute all 10 proxy metrics from a bag file."""
    frames = _extract_frames(bag_path, topic)
    n_frames = len(frames)
    if n_frames == 0:
        raise RuntimeError(
            f"No messages found on topic '{topic}' in {bag_path}. "
            f"Check topic name with: rosbag info {bag_path}"
        )

    # --- Per-frame detection counts ---
    counts = [f["n_detections"] for f in frames]
    mean_det = _safe_mean(counts)
    std_det = _safe_std(counts, mean_det)

    # Spike rate: |Δn| > 2*std between consecutive frames
    spike_threshold = 2.0 * std_det if std_det > 0 else 1.0
    n_spikes = 0
    for i in range(1, n_frames):
        if abs(counts[i] - counts[i - 1]) > spike_threshold:
            n_spikes += 1
    spike_rate = n_spikes / max(n_frames - 1, 1)

    # Zero-frame rate
    n_zero = sum(1 for c in counts if c == 0)
    zero_frame_rate = n_zero / n_frames

    # --- Confidence ---
    all_conf: List[float] = []
    for f in frames:
        all_conf.extend(f["confidences"])
    mean_conf = _safe_mean(all_conf)
    std_conf = _safe_std(all_conf, mean_conf)

    # --- Distance ---
    all_dist: List[float] = []
    for f in frames:
        all_dist.extend(f["distances"])
    mean_dist = _safe_mean(all_dist)

    # --- Symmetry ratio: min(left, right) / max(left, right) ---
    total_left = sum(f["n_left"] for f in frames)
    total_right = sum(f["n_right"] for f in frames)
    if max(total_left, total_right) > 0:
        symmetry = min(total_left, total_right) / max(total_left, total_right)
    else:
        symmetry = 0.0

    # --- Frame interval ---
    stamps = [f["stamp_sec"] for f in frames]
    intervals = [stamps[i] - stamps[i - 1] for i in range(1, len(stamps))]
    mean_interval = _safe_mean(intervals)

    result = {
        "version": 2,
        "generated_at": datetime.now().isoformat(),
        "source_bag": str(bag_path),
        "topic": topic,
        "metrics": {
            "n_frames": n_frames,
            "mean_detections": round(mean_det, 3),
            "std_detections": round(std_det, 3),
            "spike_rate": round(spike_rate, 4),
            "zero_frame_rate": round(zero_frame_rate, 4),
            "mean_confidence": round(mean_conf, 4),
            "std_confidence": round(std_conf, 4),
            "mean_distance_m": round(mean_dist, 3),
            "symmetry_ratio": round(symmetry, 4),
            "mean_frame_interval_s": round(mean_interval, 6),
        },
    }

    # GT evaluation (optional)
    if gt_path is not None:
        gt_cones = _load_gt_cones(gt_path)
        if not gt_cones:
            print(f"WARNING: no cones loaded from {gt_path}", file=sys.stderr)
        else:
            gt_metrics = _evaluate_gt(frames, gt_cones, gt_match_threshold, gt_max_range)
            result["gt_file"] = str(gt_path)
            result["metrics"].update(gt_metrics)

    return result

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Evaluate LiDAR perception proxy metrics from a ROS bag file."
    )
    parser.add_argument(
        "bag_file", type=Path, help="Path to the ROS bag file."
    )
    parser.add_argument(
        "--topic", type=str, default=DEFAULT_TOPIC,
        help=f"Detection topic to read (default: {DEFAULT_TOPIC}).",
    )
    parser.add_argument(
        "-o", "--output-json", type=Path, default=None,
        help="Output JSON path. Defaults to <bag_stem>_perception.json alongside the bag.",
    )
    parser.add_argument(
        "--gt", type=Path, default=None,
        help="Ground-truth cone map CSV (x,y,z per line). Enables precision/recall/F1.",
    )
    parser.add_argument(
        "--gt-threshold", type=float, default=1.0,
        help="GT matching distance threshold in meters (default: 1.0).",
    )
    parser.add_argument(
        "--gt-max-range", type=float, default=50.0,
        help="Max range for GT visibility filter in meters (default: 50.0).",
    )
    args = parser.parse_args()

    if not args.bag_file.exists():
        print(f"ERROR: bag file not found: {args.bag_file}", file=sys.stderr)
        return 1

    if args.gt is not None and not args.gt.exists():
        print(f"ERROR: GT file not found: {args.gt}", file=sys.stderr)
        return 1

    payload = evaluate(
        args.bag_file, args.topic,
        gt_path=args.gt,
        gt_match_threshold=args.gt_threshold,
        gt_max_range=args.gt_max_range,
    )

    # Default output path
    if args.output_json is None:
        args.output_json = args.bag_file.with_name(
            args.bag_file.stem + "_perception.json"
        )

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    # Print summary to stdout
    m = payload["metrics"]
    print(f"[OK] {m['n_frames']} frames from {args.bag_file.name}")
    print(f"     mean_det={m['mean_detections']}  std_det={m['std_detections']}")
    print(f"     spike_rate={m['spike_rate']}  zero_rate={m['zero_frame_rate']}")
    print(f"     confidence={m['mean_confidence']}±{m['std_confidence']}")
    print(f"     distance={m['mean_distance_m']}m  symmetry={m['symmetry_ratio']}")
    print(f"     interval={m['mean_frame_interval_s']}s")
    if "precision" in m:
        print(f"  GT precision={m['precision']}  recall={m['recall']}  F1={m['f1']}")
        print(f"     RMSE={m['rmse_m']}m  TP={m['total_tp']} FP={m['total_fp']} FN={m['total_fn']}")
        print(f"     gt_visible={m['mean_gt_visible']}  threshold={m['gt_match_threshold_m']}m")
    print(f"     output: {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
