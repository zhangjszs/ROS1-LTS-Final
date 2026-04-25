#!/usr/bin/env python3
"""
Generic LiDAR perception replay evaluation runner.

Usage:
    cd /home/kerwin/2025huat
    python3 scripts/run_perception_evaluation.py \
        --bag ~/rosbag/track.bag \
        --duration 60 \
        --start 30 \
        --output-dir /tmp/perception_eval

Baseline comparison:
    python3 scripts/run_perception_evaluation.py \
        --compare /tmp/perception_eval/result_a.json /tmp/perception_eval/result_b.json \
        --output-dir /tmp/perception_eval
"""

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

WORKSPACE = Path("/home/kerwin/2025huat")
BASE_YAML = WORKSPACE / "src/perception_ros/config/lidar_base.yaml"
TRACK_YAML = WORKSPACE / "src/perception_ros/config/lidar_track.yaml"
BENCHMARK_SCRIPT = WORKSPACE / "scripts/benchmark_trackdrive.py"


def _kill_ros_procs():
    """Kill any leftover roscore / roslaunch / rosbag processes."""
    for pattern in ["roscore", "rosmaster", "roslaunch", "rosbag"]:
        subprocess.run(["pkill", "-f", pattern], capture_output=True)
    time.sleep(2)


def save_config_snapshot(output_dir: Path, tag: str) -> Dict[str, str]:
    """Copy lidar_base.yaml and lidar_track.yaml into output dir with timestamp."""
    snapshot_dir = output_dir / "config_snapshot"
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    copied = {}
    for src in [BASE_YAML, TRACK_YAML]:
        if src.exists():
            dst = snapshot_dir / f"{src.stem}_{tag}{src.suffix}"
            shutil.copy2(str(src), str(dst))
            copied[src.name] = str(dst)
    return copied


def run_benchmark_subprocess(
    duration_sec: int,
    output_path: Path,
    warmup_sec: int = 10,
) -> Optional[dict]:
    """Run benchmark_trackdrive.py as a subprocess and return parsed JSON."""
    env = os.environ.copy()
    env["PYTHONPATH"] = (
        f"{WORKSPACE}/devel/lib/python3/dist-packages:"
        f"/opt/ros/noetic/lib/python3/dist-packages:"
        f"{env.get('PYTHONPATH', '')}"
    )
    benchmark_cmd = [
        "python3",
        str(BENCHMARK_SCRIPT),
        "--output",
        str(output_path),
        "--duration",
        str(duration_sec),
    ]
    bench_proc = subprocess.Popen(
        f"cd {WORKSPACE} && source devel/setup.bash && {' '.join(benchmark_cmd)}",
        shell=True,  # nosec B602
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    try:
        bench_stdout, _ = bench_proc.communicate(timeout=duration_sec + 30)
        print(bench_stdout)
    except subprocess.TimeoutExpired:
        bench_proc.kill()
        bench_proc.wait()
        print("[ERROR] Benchmark subprocess timed out")
        return None

    if output_path.exists():
        with open(output_path) as f:
            return json.load(f)
    print(f"[WARN] Benchmark output not found: {output_path}")
    return None


def run_single_evaluation(
    bag_path: str,
    duration_sec: int,
    start_sec: int,
    output_dir: Path,
    warmup_sec: int = 10,
    launch_file: str = "trackdrive.launch",
    config_snapshot: bool = False,
) -> dict:
    """
    Run one evaluation:
      1. Save config snapshot
      2. Launch ROS stack with rosbag
      3. Run benchmark
      4. Kill ROS processes
      5. Return result dict
    """
    tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    print(f"\n{'='*60}")
    print(f"EVALUATION RUN  tag={tag}")
    print(f"  bag={bag_path}")
    print(f"  duration={duration_sec}s  start={start_sec}s")
    print(f"{'='*60}")

    # 1. Config snapshot
    print("[1/4] Saving config snapshot...")
    config_snapshot_paths = save_config_snapshot(output_dir, tag) if config_snapshot else {}
    print(f"  -> {config_snapshot_paths}")

    # 2. Launch ROS stack
    print("[2/4] Launching roslaunch...")
    launch_cmd = [
        "roslaunch",
        "fsd_launch",
        launch_file,
        "simulation:=true",
        f"bag:={bag_path}",
        f"start:={start_sec}",
        "loop:=false",
        "launch_rviz:=false",
        "launch_viz:=false",
    ]
    env = os.environ.copy()
    env["PYTHONPATH"] = (
        f"{WORKSPACE}/devel/lib/python3/dist-packages:"
        f"/opt/ros/noetic/lib/python3/dist-packages:"
        f"{env.get('PYTHONPATH', '')}"
    )
    launch_proc = subprocess.Popen(
        f"cd {WORKSPACE} && source devel/setup.bash && {' '.join(launch_cmd)}",
        shell=True,  # nosec B602
        executable="/bin/bash",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=env,
    )

    result = {
        "meta": {
            "tag": tag,
            "bag_path": bag_path,
            "duration_sec": duration_sec,
            "start_sec": start_sec,
            "launch_file": launch_file,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        },
        "config_snapshot": config_snapshot_paths,
        "benchmark": None,
    }

    try:
        # 3. Warmup
        print(f"[3/4] Warmup {warmup_sec}s...")
        time.sleep(warmup_sec)

        # 4. Run benchmark
        print(f"[4/4] Running benchmark for {duration_sec}s...")
        benchmark_output = output_dir / f"benchmark_{tag}.json"
        bench = run_benchmark_subprocess(
            duration_sec=duration_sec,
            output_path=benchmark_output,
            warmup_sec=warmup_sec,
        )
        result["benchmark"] = bench

    except Exception as e:
        print(f"[ERROR] During evaluation: {e}")
        result["error"] = str(e)
    finally:
        # 5. Kill launch
        print("[CLEANUP] Killing roslaunch...")
        try:
            launch_proc.send_signal(signal.SIGINT)
            launch_proc.wait(timeout=5)
        except Exception:
            launch_proc.kill()
            launch_proc.wait()
        time.sleep(2)
        _kill_ros_procs()
        print("[DONE] ROS processes cleaned up.")

    # Save result
    result_path = output_dir / f"evaluation_{tag}.json"
    with open(result_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"[RESULT] Saved to {result_path}")
    return result


def build_comparison_md(result_a: dict, result_b: dict, label_a: str, label_b: str) -> str:
    """Build a Markdown side-by-side comparison of two evaluation results."""

    def _get(d, *keys, default=0.0):
        for key in keys:
            if isinstance(d, dict) and key in d:
                d = d[key]
            else:
                return default
        return d

    def _fmt(val, fmt=".2f"):
        try:
            return f"{val:{fmt}}"
        except (ValueError, TypeError):
            return str(val)

    def _row(metric, a_val, b_val, lower_is_better=False):
        try:
            delta = float(b_val) - float(a_val)
            if lower_is_better:
                better = "B" if delta < 0 else "A" if delta > 0 else "="
            else:
                better = "A" if delta < 0 else "B" if delta > 0 else "="
            sign = "+" if delta >= 0 else ""
            return f"| {metric} | {_fmt(a_val)} | {_fmt(b_val)} | {sign}{_fmt(delta)} | {better} |"
        except (ValueError, TypeError):
            return f"| {metric} | {a_val} | {b_val} | - | - |"

    lines = [
        f"# Perception Evaluation Comparison",
        f"",
        f"| | {label_a} | {label_b} | Delta | Better |",
        f"|---|---|---|---|---|",
    ]

    # Perception
    lines.append("| **Perception** | | | | |")
    lines.append(
        _row(
            "Frames",
            _get(result_a, "benchmark", "perception", "frames"),
            _get(result_b, "benchmark", "perception", "frames"),
        )
    )
    lines.append(
        _row(
            "Mean cones/frame",
            _get(result_a, "benchmark", "perception", "mean_cones_per_frame"),
            _get(result_b, "benchmark", "perception", "mean_cones_per_frame"),
        )
    )
    lines.append(
        _row(
            "Mean confidence",
            _get(result_a, "benchmark", "perception", "mean_confidence"),
            _get(result_b, "benchmark", "perception", "mean_confidence"),
        )
    )
    lines.append(
        _row(
            "Mean distance (m)",
            _get(result_a, "benchmark", "perception", "mean_distance_m"),
            _get(result_b, "benchmark", "perception", "mean_distance_m"),
        )
    )

    for band in ["0-10m", "10-20m", "20-30m", "30-40m", "40-50m", ">50m"]:
        a_mean = _get(result_a, "benchmark", "perception", "bands", band, "mean_per_frame")
        b_mean = _get(result_b, "benchmark", "perception", "bands", band, "mean_per_frame")
        lines.append(_row(f"  {band} mean/frame", a_mean, b_mean))

    # Planning
    lines.append("| **Planning** | | | | |")
    lines.append(
        _row(
            "Frames",
            _get(result_a, "benchmark", "planning", "frames"),
            _get(result_b, "benchmark", "planning", "frames"),
        )
    )
    lines.append(
        _row(
            "Mean path points",
            _get(result_a, "benchmark", "planning", "mean_path_points"),
            _get(result_b, "benchmark", "planning", "mean_path_points"),
        )
    )
    lines.append(
        _row(
            "Short path rate %",
            _get(result_a, "benchmark", "planning", "short_path_rate_pct"),
            _get(result_b, "benchmark", "planning", "short_path_rate_pct"),
            lower_is_better=True,
        )
    )

    # Control
    lines.append("| **Control** | | | | |")
    lines.append(
        _row(
            "Frames",
            _get(result_a, "benchmark", "control", "frames"),
            _get(result_b, "benchmark", "control", "frames"),
        )
    )
    lines.append(
        _row(
            "Delta std (rad)",
            _get(result_a, "benchmark", "control", "delta_std_rad"),
            _get(result_b, "benchmark", "control", "delta_std_rad"),
            lower_is_better=True,
        )
    )
    lines.append(
        _row(
            "Mean speed (m/s)",
            _get(result_a, "benchmark", "control", "mean_speed_m_s"),
            _get(result_b, "benchmark", "control", "mean_speed_m_s"),
        )
    )

    # System
    lines.append("| **System** | | | | |")
    lines.append(
        _row(
            "Velo Hz",
            _get(result_a, "benchmark", "system", "velo_hz"),
            _get(result_b, "benchmark", "system", "velo_hz"),
        )
    )
    lines.append(
        _row(
            "Det Hz",
            _get(result_a, "benchmark", "system", "det_hz"),
            _get(result_b, "benchmark", "system", "det_hz"),
        )
    )
    lines.append(
        _row(
            "Input pointcloud Hz",
            _get(result_a, "benchmark", "system", "input_pointcloud_hz"),
            _get(result_b, "benchmark", "system", "input_pointcloud_hz"),
        )
    )
    lines.append(
        _row(
            "Detection Hz",
            _get(result_a, "benchmark", "system", "detection_hz"),
            _get(result_b, "benchmark", "system", "detection_hz"),
        )
    )
    lines.append(
        _row(
            "Plan Hz",
            _get(result_a, "benchmark", "system", "plan_hz"),
            _get(result_b, "benchmark", "system", "plan_hz"),
        )
    )
    lines.append(
        _row(
            "Cmd Hz",
            _get(result_a, "benchmark", "system", "cmd_hz"),
            _get(result_b, "benchmark", "system", "cmd_hz"),
        )
    )
    lines.append(
        _row(
            "Mean CPU %",
            _get(result_a, "benchmark", "system", "mean_cpu_pct"),
            _get(result_b, "benchmark", "system", "mean_cpu_pct"),
            lower_is_better=True,
        )
    )
    lines.append(
        _row(
            "Mean Mem MB",
            _get(result_a, "benchmark", "system", "mean_mem_mb"),
            _get(result_b, "benchmark", "system", "mean_mem_mb"),
            lower_is_better=True,
        )
    )

    # Diagnostics
    lines.append("| **Diagnostics** | | | | |")
    lines.append(
        _row(
            "Diag frames",
            _get(result_a, "benchmark", "diagnostics", "diag_frames_received"),
            _get(result_b, "benchmark", "diagnostics", "diag_frames_received"),
        )
    )
    lines.append(
        _row(
            "Input points mean",
            _get(result_a, "benchmark", "diagnostics", "pipeline_funnel", "input_points_mean"),
            _get(result_b, "benchmark", "diagnostics", "pipeline_funnel", "input_points_mean"),
        )
    )
    lines.append(
        _row(
            "ROI dropped mean",
            _get(result_a, "benchmark", "diagnostics", "pipeline_funnel", "roi_dropped_mean"),
            _get(result_b, "benchmark", "diagnostics", "pipeline_funnel", "roi_dropped_mean"),
        )
    )
    lines.append(
        _row(
            "Ground removed mean",
            _get(result_a, "benchmark", "diagnostics", "pipeline_funnel", "ground_removed_mean"),
            _get(result_b, "benchmark", "diagnostics", "pipeline_funnel", "ground_removed_mean"),
        )
    )
    lines.append(
        _row(
            "Clusters total mean",
            _get(result_a, "benchmark", "diagnostics", "pipeline_funnel", "clusters_total_mean"),
            _get(result_b, "benchmark", "diagnostics", "pipeline_funnel", "clusters_total_mean"),
        )
    )
    lines.append(
        _row(
            "Clusters far mean",
            _get(result_a, "benchmark", "diagnostics", "pipeline_funnel", "clusters_far_mean"),
            _get(result_b, "benchmark", "diagnostics", "pipeline_funnel", "clusters_far_mean"),
        )
    )
    lines.append(
        _row(
            "Rej total",
            _get(result_a, "benchmark", "diagnostics", "rejection_reasons", "total"),
            _get(result_b, "benchmark", "diagnostics", "rejection_reasons", "total"),
            lower_is_better=True,
        )
    )
    lines.append(
        _row(
            "Rej by confidence",
            _get(result_a, "benchmark", "diagnostics", "rejection_reasons", "by_confidence"),
            _get(result_b, "benchmark", "diagnostics", "rejection_reasons", "by_confidence"),
            lower_is_better=True,
        )
    )
    lines.append(
        _row(
            "Rej by semantic",
            _get(result_a, "benchmark", "diagnostics", "rejection_reasons", "by_semantic"),
            _get(result_b, "benchmark", "diagnostics", "rejection_reasons", "by_semantic"),
            lower_is_better=True,
        )
    )
    lines.append(
        _row(
            "Size score",
            _get(result_a, "benchmark", "diagnostics", "confidence_components", "size"),
            _get(result_b, "benchmark", "diagnostics", "confidence_components", "size"),
        )
    )
    lines.append(
        _row(
            "Shape score",
            _get(result_a, "benchmark", "diagnostics", "confidence_components", "shape"),
            _get(result_b, "benchmark", "diagnostics", "confidence_components", "shape"),
        )
    )
    lines.append(
        _row(
            "Density score",
            _get(result_a, "benchmark", "diagnostics", "confidence_components", "density"),
            _get(result_b, "benchmark", "diagnostics", "confidence_components", "density"),
        )
    )
    lines.append(
        _row(
            "Semantic score",
            _get(result_a, "benchmark", "diagnostics", "confidence_components", "semantic"),
            _get(result_b, "benchmark", "diagnostics", "confidence_components", "semantic"),
        )
    )
    lines.append(
        _row(
            "Fitting success rate %",
            _get(result_a, "benchmark", "diagnostics", "model_fitting", "success_rate_pct"),
            _get(result_b, "benchmark", "diagnostics", "model_fitting", "success_rate_pct"),
        )
    )
    lines.append(
        _row(
            "Tracker confirmed",
            _get(result_a, "benchmark", "diagnostics", "tracker", "confirmed_mean"),
            _get(result_b, "benchmark", "diagnostics", "tracker", "confirmed_mean"),
        )
    )
    lines.append(
        _row(
            "Total p95 ms",
            _get(result_a, "benchmark", "diagnostics", "performance", "total_p95_ms"),
            _get(result_b, "benchmark", "diagnostics", "performance", "total_p95_ms"),
            lower_is_better=True,
        )
    )
    lines.append(
        _row(
            "Ground p95 ms",
            _get(result_a, "benchmark", "diagnostics", "performance", "ground_p95_ms"),
            _get(result_b, "benchmark", "diagnostics", "performance", "ground_p95_ms"),
            lower_is_better=True,
        )
    )

    lines.append("")
    lines.append("**Legend**: A = first result is better, B = second result is better, = = tie.")
    lines.append("")
    return "\n".join(lines)


def compare_results(path_a: str, path_b: str, output_dir: Path) -> None:
    """Load two evaluation JSONs and produce a side-by-side Markdown report."""
    with open(path_a) as f:
        result_a = json.load(f)
    with open(path_b) as f:
        result_b = json.load(f)

    label_a = result_a.get("meta", {}).get("tag", "A")
    label_b = result_b.get("meta", {}).get("tag", "B")

    md = build_comparison_md(result_a, result_b, label_a, label_b)
    tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    comparison_path = output_dir / f"comparison_{label_a}_vs_{label_b}_{tag}.md"
    with open(comparison_path, "w") as f:
        f.write(md)
    print(f"\n[COMPARISON] Saved to {comparison_path}")

    # Also write JSON diff
    diff = {
        "label_a": label_a,
        "label_b": label_b,
        "path_a": path_a,
        "path_b": path_b,
        "result_a": result_a,
        "result_b": result_b,
    }
    diff_path = output_dir / f"comparison_{label_a}_vs_{label_b}_{tag}.json"
    with open(diff_path, "w") as f:
        json.dump(diff, f, indent=2)
    print(f"[COMPARISON JSON] Saved to {diff_path}")


def main():
    parser = argparse.ArgumentParser(description="Generic LiDAR perception replay evaluation")
    parser.add_argument(
        "--bag", type=str, default="/home/kerwin/rosbag/track.bag", help="Rosbag path"
    )
    parser.add_argument(
        "--duration", type=int, default=60, help="Benchmark duration per run (seconds)"
    )
    parser.add_argument("--start", type=int, default=30, help="Rosbag start offset in seconds")
    parser.add_argument(
        "--output-dir",
        type=str,
        default="/tmp/perception_eval",  # nosec B108
        help="Output directory",
    )
    parser.add_argument(
        "--warmup", type=int, default=10, help="Seconds to wait after launch before benchmark"
    )
    parser.add_argument(
        "--launch", type=str, default="trackdrive.launch", help="Launch file to use"
    )
    parser.add_argument(
        "--compare",
        nargs=2,
        metavar=("RESULT_A", "RESULT_B"),
        help="Compare two evaluation JSONs side-by-side",
    )
    parser.add_argument("--skip-build", action="store_true", help="Skip catkin build")
    parser.add_argument(
        "--config-snapshot",
        action="store_true",
        help="Copy YAML config files into output directory",
    )
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.compare:
        compare_results(args.compare[0], args.compare[1], output_dir)
        return

    if not Path(args.bag).exists():
        print(f"[ERROR] Rosbag not found: {args.bag}")
        sys.exit(1)

    if not args.skip_build:
        print("[BUILD] Running catkin build...")
        build_proc = subprocess.run(
            "cd /home/kerwin/2025huat && catkin build perception_core perception_ros",
            shell=True,
            executable="/bin/bash",
            capture_output=True,
            text=True,
        )
        if build_proc.returncode != 0:
            print(f"[ERROR] Build failed:\n{build_proc.stderr}")
            sys.exit(1)
        print("[BUILD] OK")

    result = run_single_evaluation(
        bag_path=args.bag,
        duration_sec=args.duration,
        start_sec=args.start,
        output_dir=output_dir,
        warmup_sec=args.warmup,
        launch_file=args.launch,
        config_snapshot=args.config_snapshot,
    )

    print("\n" + "=" * 60)
    print("EVALUATION COMPLETE")
    print("=" * 60)
    bench = result.get("benchmark")
    if bench:
        print(json.dumps(bench, indent=2))
    else:
        print("[WARN] No benchmark data collected.")


if __name__ == "__main__":
    main()
