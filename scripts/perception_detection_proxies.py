#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compute lightweight proxy metrics from a recorded detections bag.

Default topic is /perception/lidar_cluster/detections (autodrive_msgs/HUAT_ConeDetections).

This script is intentionally simple: it does not require replaying the full stack.
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict


DEFAULT_TOPIC = "/perception/lidar_cluster/detections"


def _compute(bag_path: Path, topic: str, far_dist: float, wall_y: float, front_y: float) -> Dict[str, Any]:
    try:
        import rosbag  # type: ignore
    except ImportError as e:
        raise SystemExit(
            "ERROR: rosbag Python package not found. Try:\n"
            "  source /opt/ros/noetic/setup.bash\n"
            "  (and/or) source your workspace setup.*\n"
        ) from e

    n_frames = 0
    total = 0
    far = 0
    wall_like = 0
    far_wall_like = 0
    far_front_like = 0

    with rosbag.Bag(str(bag_path), "r") as bag:
        for _topic, msg, _t in bag.read_messages(topics=[topic]):
            n_frames += 1
            for p in msg.points:
                d = math.hypot(float(p.x), float(p.y))
                total += 1

                if abs(float(p.y)) >= wall_y:
                    wall_like += 1

                if d >= far_dist:
                    far += 1
                    if abs(float(p.y)) >= wall_y:
                        far_wall_like += 1
                    if abs(float(p.y)) <= front_y:
                        far_front_like += 1

    def ratio(num: int, den: int) -> float:
        return (num / den) if den else 0.0

    return {
        "bag": str(bag_path),
        "topic": topic,
        "params": {
            "far_dist_m": far_dist,
            "wall_abs_y_m": wall_y,
            "front_abs_y_m": front_y,
        },
        "counts": {
            "n_frames": n_frames,
            "total_detections": total,
            "wall_like_abs_y_ge_wall_y": wall_like,
            "far_detections_dist_ge_far_dist": far,
            "far_wall_like_dist_ge_far_dist_abs_y_ge_wall_y": far_wall_like,
            "far_front_like_dist_ge_far_dist_abs_y_le_front_y": far_front_like,
        },
        "ratios": {
            "ratio_wall_like": ratio(wall_like, total),
            "ratio_far_wall_like": ratio(far_wall_like, far),
            "ratio_far_front_like": ratio(far_front_like, far),
        },
        "per_frame": {
            "total_det_per_frame": ratio(total, n_frames),
            "far_det_per_frame": ratio(far, n_frames),
            "far_wall_like_per_frame": ratio(far_wall_like, n_frames),
            "far_front_like_per_frame": ratio(far_front_like, n_frames),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="Recorded detections bag path")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help=f"Detections topic (default: {DEFAULT_TOPIC})")
    parser.add_argument("--far-dist", type=float, default=18.0, help="Far distance threshold [m]")
    parser.add_argument("--wall-y", type=float, default=6.0, help="Wall-like abs(y) threshold [m]")
    parser.add_argument("--front-y", type=float, default=1.0, help="Front-like abs(y) threshold [m]")
    parser.add_argument("-o", "--output", type=Path, default=None, help="Write JSON output to file")
    args = parser.parse_args()

    if not args.bag.exists():
        print(f"ERROR: bag not found: {args.bag}", file=sys.stderr)
        return 2

    out = _compute(args.bag, args.topic, args.far_dist, args.wall_y, args.front_y)
    payload = json.dumps(out, indent=2, ensure_ascii=False)
    if args.output:
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

