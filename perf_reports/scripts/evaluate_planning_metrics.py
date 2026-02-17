#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Planning evaluation — proxy metrics from bag replay.

Usage:
    python3 evaluate_planning_metrics.py <bag_file> [-o output.json]

Proxy metrics:
   1. n_frames              — total frames with pathlimits published
   2. mean_path_size        — average path points per frame
   3. std_path_size         — std dev of path size (stability)
   4. path_size_min         — minimum path size
   5. path_size_max         — maximum path size
   6. mean_curvature        — average curvature magnitude [1/m]
   7. max_curvature         — maximum curvature [1/m]
   8. curvature_std         — std dev of curvature
   9. mean_target_speed     — average target speed [m/s]
   10. max_target_speed     — maximum target speed [m/s]
   11. speed_std             — std dev of target speed
   12. replan_rate           — fraction of frames where replan=true
   13. replan_count          — total frames with replan=true
   14. mean_path_jump       — average position jump between consecutive frames [m]
   15. max_path_jump        — maximum position jump [m]
   16. spike_jump_rate       — fraction of frames with jump > 1.0m (path instability)
   17. empty_path_rate      — fraction of frames with empty path
   18. mean_frame_interval_s — average time between frames [s]
   19. mission_type          — inferred mission type (line/skidpad/high_speed)

Requires:
    pip install rosbag rosmsg geometry_msgs
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

DEFAULT_TOPIC = "/planning/pathlimits"


def _safe_mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _safe_std(values: List[float], mean: float) -> float:
    if len(values) < 2:
        return 0.0
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def _distance_2d(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def _load_pathlimits_from_bag(bag_path: Path, topic: str) -> List[Dict[str, Any]]:
    """Load pathlimits messages from rosbag."""
    try:
        import rosbag
    except ImportError:
        print("ERROR: rosbag not installed. Install with: pip install rosbag", file=sys.stderr)
        sys.exit(1)

    frames: List[Dict[str, Any]] = []

    try:
        bag = rosbag.Bag(str(bag_path), 'r')
    except Exception as e:
        print(f"ERROR: Failed to open bag: {e}", file=sys.stderr)
        sys.exit(1)

        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            try:
                path = msg.path
                curvatures = msg.curvatures if hasattr(msg, 'curvatures') else []
                target_speeds = msg.target_speeds if hasattr(msg, 'target_speeds') else []
                replan = msg.replan if hasattr(msg, 'replan') else False

                frame = {
                    "stamp": msg.header.stamp.to_sec() if hasattr(msg.header, 'stamp') else t.to_sec(),
                    "path_size": len(path),
                    "path_x": [p.x for p in path],
                    "path_y": [p.y for p in path],
                    "curvatures": curvatures,
                    "target_speeds": target_speeds,
                    "replan": replan,
                }
                frames.append(frame)
            except Exception as e:
                print(f"WARN: Failed to parse message: {e}", file=sys.stderr)
                continue

        bag.close()

    return frames


def _compute_path_jumps(frames: List[Dict[str, Any]]) -> List[float]:
    """Compute position jumps between consecutive frames."""
    jumps = []
    for i in range(1, len(frames)):
        prev_path = frames[i - 1]["path_x"]
        curr_path = frames[i]["path_x"]

        if not prev_path or not curr_path:
            continue

        jump = _distance_2d(
            (prev_path[0], frames[i - 1]["path_y"][0]),
            (curr_path[0], frames[i]["path_y"][0])
        )
        jumps.append(jump)

    return jumps


def _infer_mission(frames: List[Dict[str, Any]]) -> str:
    """Infer mission type from path characteristics."""
    if not frames:
        return "unknown"

    mean_size = _safe_mean([f["path_size"] for f in frames])
    mean_speed = _safe_mean([
        _safe_mean(f["target_speeds"]) for f in frames if f["target_speeds"]
    ])

    if mean_speed > 15.0:
        return "high_speed"
    elif mean_speed > 5.0 and mean_speed <= 15.0:
        return "skidpad"
    elif mean_size < 50:
        return "acceleration"
    else:
        return "line"


def evaluate_planning_metrics(frames: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Compute all proxy metrics from pathlimits frames."""
    if not frames:
        return {"error": "No frames extracted from bag"}

    n_frames = len(frames)

    path_sizes = [f["path_size"] for f in frames]
    mean_path_size = _safe_mean(path_sizes)
    std_path_size = _safe_std(path_sizes, mean_path_size)

    all_curvatures = []
    for f in frames:
        all_curvatures.extend([abs(c) for c in f.get("curvatures", [])])

    mean_curvature = _safe_mean(all_curvatures)
    max_curvature = max(all_curvatures) if all_curvatures else 0.0
    curvature_std = _safe_std(all_curvatures, mean_curvature)

    all_speeds = []
    for f in frames:
        all_speeds.extend(f.get("target_speeds", []))

    mean_target_speed = _safe_mean(all_speeds)
    max_target_speed = max(all_speeds) if all_speeds else 0.0
    speed_std = _safe_std(all_speeds, mean_target_speed)

    replan_count = sum(1 for f in frames if f.get("replan", False))
    replan_rate = replan_count / n_frames if n_frames > 0 else 0.0

    empty_path_count = sum(1 for f in frames if f["path_size"] == 0)
    empty_path_rate = empty_path_count / n_frames if n_frames > 0 else 0.0

    jumps = _compute_path_jumps(frames)
    mean_path_jump = _safe_mean(jumps) if jumps else 0.0
    max_path_jump = max(jumps) if jumps else 0.0
    spike_jump_count = sum(1 for j in jumps if j > 1.0)
    spike_jump_rate = spike_jump_count / len(jumps) if jumps else 0.0

    timestamps = [f["stamp"] for f in frames]
    intervals = []
    for i in range(1, len(timestamps)):
        intervals.append(timestamps[i] - timestamps[i - 1])
    mean_frame_interval = _safe_mean(intervals) if intervals else 0.0

    mission = _infer_mission(frames)

    return {
        "n_frames": n_frames,
        "mean_path_size": round(mean_path_size, 2),
        "std_path_size": round(std_path_size, 2),
        "path_size_min": min(path_sizes) if path_sizes else 0,
        "path_size_max": max(path_sizes) if path_sizes else 0,
        "mean_curvature": round(mean_curvature, 6),
        "max_curvature": round(max_curvature, 6),
        "curvature_std": round(curvature_std, 6),
        "mean_target_speed": round(mean_target_speed, 2),
        "max_target_speed": round(max_target_speed, 2),
        "speed_std": round(speed_std, 2),
        "replan_rate": round(replan_rate, 4),
        "replan_count": replan_count,
        "mean_path_jump": round(mean_path_jump, 3),
        "max_path_jump": round(max_path_jump, 3),
        "spike_jump_rate": round(spike_jump_rate, 4),
        "empty_path_rate": round(empty_path_rate, 4),
        "empty_path_count": empty_path_count,
        "mean_frame_interval_s": round(mean_frame_interval, 4),
        "mission_type": mission,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate planning proxy metrics from rosbag"
    )
    parser.add_argument("bag", help="Path to rosbag file")
    parser.add_argument(
        "-t", "--topic", default=DEFAULT_TOPIC,
        help=f"PathLimits topic (default: {DEFAULT_TOPIC})"
    )
    parser.add_argument(
        "-o", "--output", default=None,
        help="Output JSON file (default: stdout)"
    )
    args = parser.parse_args()

    bag_path = Path(args.bag)
    if not bag_path.exists():
        print(f"ERROR: Bag file not found: {bag_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading pathlimits from {args.topic}...")
    frames = _load_pathlimits_from_bag(bag_path, args.topic)
    print(f"Loaded {len(frames)} frames")

    if not frames:
        print("ERROR: No pathlimits messages found in bag", file=sys.stderr)
        sys.exit(1)

    metrics = evaluate_planning_metrics(frames)

    output = {
        "bag_file": str(bag_path),
        "topic": args.topic,
        "metrics": metrics,
    }

    if args.output:
        with open(args.output, "w") as f:
            json.dump(output, f, indent=2)
        print(f"Results written to {args.output}")
    else:
        print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
