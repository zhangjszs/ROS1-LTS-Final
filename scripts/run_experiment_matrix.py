#!/usr/bin/env python3
"""
Task #20: Far-range threshold grid experiment runner.
Automates parameter patching, rosbag replay, benchmark collection,
and result aggregation.

Usage:
    cd /home/kerwin/2025huat
    python3 scripts/run_experiment_matrix.py --duration 60 --bag ~/rosbag/track.bag
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

try:
    import yaml
except ImportError:
    print("[ERROR] PyYAML not installed. Run: pip install pyyaml")
    sys.exit(1)


WORKSPACE = Path("/home/kerwin/2025huat")
BASE_YAML = WORKSPACE / "src/perception_ros/config/lidar_base.yaml"
TRACK_YAML = WORKSPACE / "src/perception_ros/config/lidar_track.yaml"
BENCHMARK_SCRIPT = WORKSPACE / "scripts/benchmark_trackdrive.py"


@dataclass
class ExperimentConfig:
    name: str
    min_neighbors_hard: int = 1
    min_confidence_far: float = 0.45
    cluster_tolerance_far_profile: str = "current"  # "current" or "relaxed_far"
    # cluster_tolerance is derived from profile


# Pre-defined cluster tolerance profiles (track.yaml segment count = 7)
# distance_segments in track.yaml: [5.0, 10.0, 15.0, 25.0, 35.0, 50.0]
# -> 7 tolerances for ranges: <5, 5-10, 10-15, 15-25, 25-35, 35-50, >=50
CLUSTER_PROFILES = {
    "current": [0.15, 0.25, 0.28, 0.35, 0.38, 0.40, 0.42],
    "relaxed_far": [0.15, 0.25, 0.28, 0.35, 0.42, 0.45, 0.45],
}


def build_matrix() -> List[ExperimentConfig]:
    """Build the 2 x 3 x 2 = 12 experiment matrix."""
    configs = []
    for neighbors in [1, 2]:
        for far_conf in [0.40, 0.45, 0.50]:
            for profile in ["current", "relaxed_far"]:
                name = f"n{neighbors}_cf{int(far_conf*100)}_{profile}"
                configs.append(
                    ExperimentConfig(
                        name=name,
                        min_neighbors_hard=neighbors,
                        min_confidence_far=far_conf,
                        cluster_tolerance_far_profile=profile,
                    )
                )
    return configs


def read_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def write_yaml(path: Path, data: dict) -> None:
    # Use default_flow_style=False for readable output, but keep lists inline for compactness
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)


def patch_track_yaml(config: ExperimentConfig) -> dict:
    """
    Patch lidar_track.yaml with experiment overrides.
    Returns the original mode_presets section so we can restore later.
    """
    data = read_yaml(TRACK_YAML)
    orig_mode_presets = json.dumps(data.get("mode_presets", {}))

    if "mode_presets" not in data:
        data["mode_presets"] = {}
    if "track" not in data["mode_presets"]:
        data["mode_presets"]["track"] = {}
    track_mp = data["mode_presets"]["track"]

    # confidence overrides
    if "confidence" not in track_mp:
        track_mp["confidence"] = {}
    track_mp["confidence"]["min_confidence_far"] = config.min_confidence_far

    if "track_semantic" not in track_mp["confidence"]:
        track_mp["confidence"]["track_semantic"] = {}
    track_mp["confidence"]["track_semantic"]["min_neighbors_hard"] = config.min_neighbors_hard

    # cluster overrides
    if "cluster" not in track_mp:
        track_mp["cluster"] = {}
    track_mp["cluster"]["cluster_tolerance"] = CLUSTER_PROFILES[
        config.cluster_tolerance_far_profile
    ]

    write_yaml(TRACK_YAML, data)
    return {"mode_presets": json.loads(orig_mode_presets)}


def restore_track_yaml(original: dict) -> None:
    data = read_yaml(TRACK_YAML)
    if "mode_presets" in original:
        data["mode_presets"] = original["mode_presets"]
    write_yaml(TRACK_YAML, data)


def run_single_experiment(
    config: ExperimentConfig,
    duration_sec: int,
    bag_path: str,
    start_sec: int = 30,
    output_dir: Path = Path("/tmp"),  # nosec B108
    warmup_sec: int = 10,
) -> dict:
    """
    Run one experiment configuration:
      1. Patch parameters
      2. Launch ROS stack with rosbag
      3. Run benchmark
      4. Restore parameters
      5. Return parsed benchmark JSON
    """
    print(f"\n{'='*60}")
    print(f"EXPERIMENT: {config.name}")
    print(f"  min_neighbors_hard={config.min_neighbors_hard}")
    print(f"  min_confidence_far={config.min_confidence_far}")
    print(f"  cluster_tolerance={config.cluster_tolerance_far_profile}")
    print(f"{'='*60}")

    # 1. Patch
    print("[1/5] Patching parameters...")
    original_data = patch_track_yaml(config)

    # 2. Launch ROS stack
    print("[2/5] Launching roslaunch...")
    launch_cmd = [
        "roslaunch",
        "fsd_launch",
        "trackdrive.launch",
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
    # Source workspace setup.bash via bash -c
    launch_proc = subprocess.Popen(
        f"cd {WORKSPACE} && source devel/setup.bash && {' '.join(launch_cmd)}",
        shell=True,  # nosec B602
        executable="/bin/bash",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=env,
    )

    try:
        # 3. Wait for warmup
        print(f"[3/5] Warmup {warmup_sec}s...")
        time.sleep(warmup_sec)

        # 4. Run benchmark
        print(f"[4/5] Running benchmark for {duration_sec}s...")
        benchmark_output = output_dir / f"benchmark_{config.name}.json"
        benchmark_cmd = [
            "python3",
            str(BENCHMARK_SCRIPT),
            "--output",
            str(benchmark_output),
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
        bench_stdout, _ = bench_proc.communicate(timeout=duration_sec + 30)
        print(bench_stdout)

        # Parse result
        result = {}
        if benchmark_output.exists():
            with open(benchmark_output) as f:
                result = json.load(f)
        else:
            print(f"[WARN] Benchmark output not found: {benchmark_output}")

    except subprocess.TimeoutExpired:
        print("[ERROR] Benchmark timed out")
        result = {"error": "timeout"}
    except Exception as e:
        print(f"[ERROR] During benchmark: {e}")
        result = {"error": str(e)}
    finally:
        # 5. Kill launch and restore
        print("[5/5] Killing roslaunch and restoring parameters...")
        try:
            launch_proc.send_signal(signal.SIGINT)
            launch_proc.wait(timeout=5)
        except Exception:
            launch_proc.kill()
            launch_proc.wait()
        # Give ROS a moment to clean up
        time.sleep(2)
        # Kill any remaining rosmaster/rosbag processes
        subprocess.run(
            "pkill -f rosbag; pkill -f roslaunch; pkill -f rosrun", shell=True, capture_output=True
        )
        time.sleep(1)
        restore_track_yaml(original_data)
        print(f"[DONE] Parameters restored.")

    # Attach experiment metadata
    result["_experiment"] = asdict(config)
    result["_experiment"]["cluster_tolerance_values"] = CLUSTER_PROFILES[
        config.cluster_tolerance_far_profile
    ]
    return result


def aggregate_results(results: List[dict], output_dir: Path) -> None:
    """Generate a Markdown summary table from all experiment results."""
    summary_md = output_dir / "experiment_summary.md"
    summary_json = output_dir / "experiment_summary.json"

    with open(summary_json, "w") as f:
        json.dump(results, f, indent=2)

    lines = [
        "# Task #20: Far-Range Threshold Grid Experiment Summary",
        "",
        f"- Duration per run: {results[0].get('meta', {}).get('duration_sec', 'N/A')}s",
        f"- Total experiments: {len(results)}",
        "",
        "## Perception Metrics",
        "",
        "| Name | neighbors | far_conf | cluster_profile | cones/frame | conf | 30-50m/frame | >50m/frame | zero_rate% | spike_rate | sym_ratio |",
        "|------|-----------|----------|-----------------|-------------|------|--------------|------------|------------|------------|-----------|",
    ]

    def _get_band(r, band_key):
        return r.get("perception", {}).get("bands", {}).get(band_key, {}).get("mean_per_frame", 0)

    def _calc_symmetry_ratio(r):
        # Simple proxy: if we had left/right counts we'd use them.
        # Fallback: use 1.0 - normalized std deviation as a rough stability proxy.
        std = r.get("perception", {}).get("std_cones_per_frame", 0)
        mean = r.get("perception", {}).get("mean_cones_per_frame", 1)
        if mean <= 0:
            return 0
        return round(max(0, 1.0 - std / mean), 3)

    def _calc_spike_rate(r):
        std = r.get("perception", {}).get("std_cones_per_frame", 0)
        mean = r.get("perception", {}).get("mean_cones_per_frame", 1)
        if mean <= 0:
            return 0
        return round(std / mean, 3)

    def _calc_zero_rate(r):
        frames = r.get("perception", {}).get("frames", 0)
        if frames <= 0:
            return 0
        # We don't have zero_frame count directly; approximate via band data
        # Use a heuristic: if mean_cones_per_frame is very low relative to typical
        # For now, leave as "-" since benchmark_trackdrive.py doesn't track zero frames explicitly
        return "-"

    for r in results:
        exp = r.get("_experiment", {})
        p = r.get("perception", {})
        name = exp.get("name", "unknown")
        neighbors = exp.get("min_neighbors_hard", "-")
        far_conf = exp.get("min_confidence_far", "-")
        profile = exp.get("cluster_tolerance_far_profile", "-")
        cones = p.get("mean_cones_per_frame", 0)
        conf = p.get("mean_confidence", 0)
        b3050 = _get_band(r, "30-50m")
        b50 = _get_band(r, ">50m")
        zero_r = _calc_zero_rate(r)
        spike = _calc_spike_rate(r)
        sym = _calc_symmetry_ratio(r)

        lines.append(
            f"| {name} | {neighbors} | {far_conf} | {profile} | {cones} | {conf} | {b3050} | {b50} | {zero_r} | {spike} | {sym} |"
        )

    lines.extend(
        [
            "",
            "## Planning & Control Metrics",
            "",
            "| Name | path_points | short_path% | delta_std | mean_speed |",
            "|------|-------------|-------------|-----------|------------|",
        ]
    )

    for r in results:
        exp = r.get("_experiment", {})
        name = exp.get("name", "unknown")
        pln = r.get("planning", {})
        ctrl = r.get("control", {})
        lines.append(
            f"| {name} | {pln.get('mean_path_points', 0)} | {pln.get('short_path_rate_pct', 0)} | "
            f"{ctrl.get('delta_std_rad', 0)} | {ctrl.get('mean_speed_m_s', 0)} |"
        )

    lines.extend(
        [
            "",
            "## System Metrics",
            "",
            "| Name | velo_hz | det_hz | plan_hz | cmd_hz | cpu% | mem_mb |",
            "|------|---------|--------|---------|--------|------|--------|",
        ]
    )

    for r in results:
        exp = r.get("_experiment", {})
        name = exp.get("name", "unknown")
        sys = r.get("system", {})
        lines.append(
            f"| {name} | {sys.get('velo_hz', 0)} | {sys.get('det_hz', 0)} | "
            f"{sys.get('plan_hz', 0)} | {sys.get('cmd_hz', 0)} | {sys.get('mean_cpu_pct', 0)} | {sys.get('mean_mem_mb', 0)} |"
        )

    lines.extend(
        [
            "",
            "## Selection Notes",
            "",
            "- **Best candidate**: Look for highest `30-50m/frame` without large `spike_rate` increase.",
            "- **Reject if**: `spike_rate` > 1.2x baseline, or `short_path%` rises significantly.",
            "- **If no good candidate**: Problem likely requires motion distortion compensation, not just thresholds.",
            "",
        ]
    )

    with open(summary_md, "w") as f:
        f.write("\n".join(lines))

    print(f"\n[SUMMARY] Saved to {summary_md} and {summary_json}")


def main():
    parser = argparse.ArgumentParser(description="Task #20: Far-range grid experiment runner")
    parser.add_argument(
        "--duration", type=int, default=60, help="Benchmark duration per run (seconds)"
    )
    parser.add_argument(
        "--bag", type=str, default="/home/kerwin/rosbag/track.bag", help="Rosbag path"
    )
    parser.add_argument("--start", type=int, default=30, help="Rosbag start offset in seconds")
    parser.add_argument(
        "--output-dir",
        type=str,
        default="/tmp/task20_experiments",  # nosec B108
        help="Output directory",
    )
    parser.add_argument(
        "--warmup", type=int, default=10, help="Seconds to wait after launch before benchmark"
    )
    parser.add_argument(
        "--baseline-only", action="store_true", help="Run only baseline (current config)"
    )
    parser.add_argument(
        "--skip-build", action="store_true", help="Skip catkin build (assume already built)"
    )
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Verify workspace
    if not WORKSPACE.exists():
        print(f"[ERROR] Workspace not found: {WORKSPACE}")
        sys.exit(1)

    if not Path(args.bag).exists():
        print(f"[ERROR] Rosbag not found: {args.bag}")
        sys.exit(1)

    # Build if needed
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

    # Build matrix
    if args.baseline_only:
        matrix = [ExperimentConfig(name="baseline")]
    else:
        matrix = build_matrix()

    print(f"\n[INFO] Running {len(matrix)} experiments, {args.duration}s each")
    print(
        f"[INFO] Estimated total time: ~{len(matrix) * (args.duration + args.warmup + 15) / 60:.0f} minutes"
    )
    print(f"[INFO] Output dir: {output_dir}\n")

    results = []
    for i, config in enumerate(matrix, 1):
        print(f"\n>>> [{i}/{len(matrix)}] Starting {config.name}...")
        result = run_single_experiment(
            config=config,
            duration_sec=args.duration,
            bag_path=args.bag,
            start_sec=args.start,
            output_dir=output_dir,
            warmup_sec=args.warmup,
        )
        results.append(result)
        # Small cooldown between runs
        time.sleep(3)

    # Aggregate
    print("\n" + "=" * 60)
    print("AGGREGATING RESULTS...")
    print("=" * 60)
    aggregate_results(results, output_dir)

    print("\n[DONE] All experiments complete.")


if __name__ == "__main__":
    main()
