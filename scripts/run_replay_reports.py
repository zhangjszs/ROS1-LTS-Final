#!/usr/bin/env python3
"""
Replay both report suites (vision_backlog_probe + legacy_timing_review) against
the three standard mission rosbags and produce updated JSON + MD findings.

Usage (must source ROS + workspace first):
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    python3 scripts/run_replay_reports.py [--outdir perf_reports/data]
"""

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Mission catalogue
# ---------------------------------------------------------------------------

MISSIONS = [
    {
        "name": "trackdrive",
        "launch": "trackdrive.launch",
        "bag": "/home/kerwin/rosbag/track.bag",
        "image_topic": "/resize_img_out",
        "budget_sec": 0.12,
        "in_backlog_probe": True,
        "in_timing_review": True,
        "input_image_hz_assumed": 19.8,
    },
    {
        "name": "autocross",
        "launch": "autocross.launch",
        "bag": "/home/kerwin/rosbag/track.bag",
        "image_topic": "/resize_img_out",
        "budget_sec": 0.12,
        "in_backlog_probe": False,
        "in_timing_review": True,
        "input_image_hz_assumed": 19.8,
    },
    {
        "name": "skidpad",
        "launch": "skidpad.launch",
        "bag": "/home/kerwin/rosbag/skidpad.bag",
        "image_topic": "/pylon_camera_node/image_raw",
        "budget_sec": 0.15,
        "in_backlog_probe": True,
        "in_timing_review": True,
        "input_image_hz_assumed": 3.32,
    },
]

# Topics to record for combined bag (covers both suites)
RECORD_TOPICS = [
    "/perception/vision/detections",
    "/perception/vision/diagnostics",
    "/perception/decision/trace",
    "/perception/diagnostics",
    "/perception/fusion/detections",
]

BAG_TIMEOUT_SEC = 55  # kill roslaunch after this many seconds (10s warmup + 40s record + 5s flush)
RECORD_INIT_DELAY_SEC = 10  # match baseline ~10s init delay to skip ONNX JIT warmup frames


# ---------------------------------------------------------------------------
# Subprocess helpers
# ---------------------------------------------------------------------------


def _kill_ros_procs():
    """Kill any leftover roscore / roslaunch / rosbag processes."""
    for pattern in ["roscore", "rosmaster", "roslaunch", "rosbag"]:
        subprocess.run(["pkill", "-f", pattern], capture_output=True)
    time.sleep(2)


def run_mission_replay(mission: dict, outdir: str) -> Optional[str]:
    """
    Launch one mission with rosbag playback and record the combined topics.
    Returns path to recorded bag on success, None on failure.
    """
    # Clear leftover ROS processes before starting the next mission replay.
    _kill_ros_procs()

    os.makedirs(outdir, exist_ok=True)
    ros_log_dir = os.path.join(outdir, "roslog")
    ros_home_dir = os.path.join(outdir, "ros_home")
    os.makedirs(ros_log_dir, exist_ok=True)
    os.makedirs(ros_home_dir, exist_ok=True)

    env = os.environ.copy()
    env["ROS_LOG_DIR"] = ros_log_dir
    env["ROS_HOME"] = ros_home_dir

    launch_cmd = [
        "roslaunch",
        "fsd_launch",
        mission["launch"],
        "simulation:=true",
        f"bag:={mission['bag']}",
        "rate:=1.0",
        "loop:=false",
        "launch_rviz:=false",
        "launch_viz:=false",
        f"vision_image_topic:={mission['image_topic']}",
    ]

    bag_out = os.path.join(outdir, f"{mission['name']}_combined.bag")
    record_cmd = [
        "rosbag",
        "record",
        "-O",
        bag_out,
    ] + RECORD_TOPICS

    launch_log_path = os.path.join(outdir, "launch.log")
    record_log_path = os.path.join(outdir, "record.log")

    print(f"  [launch] {' '.join(launch_cmd)}")
    with open(launch_log_path, "w") as launch_log, open(record_log_path, "w") as record_log:
        launch_proc = subprocess.Popen(
            launch_cmd, stdout=launch_log, stderr=subprocess.STDOUT, env=env
        )
        # Small wait for nodes to spin up before starting the recorder.
        time.sleep(RECORD_INIT_DELAY_SEC)

        record_proc = subprocess.Popen(
            record_cmd, stdout=record_log, stderr=subprocess.STDOUT, env=env
        )

        try:
            launch_proc.wait(timeout=BAG_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            print(f"  [warn] launch timed out after {BAG_TIMEOUT_SEC}s – killing")
            launch_proc.kill()
            launch_proc.wait()

        # Give rosbag record a moment to flush then stop it gracefully.
        time.sleep(3)
        try:
            record_proc.send_signal(signal.SIGINT)
            record_proc.wait(timeout=15)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            record_proc.kill()
            record_proc.wait()

    _kill_ros_procs()

    if not os.path.exists(bag_out):
        print(f"  [error] recorded bag not found: {bag_out}")
        return None

    bag_size = os.path.getsize(bag_out)
    print(f"  [ok] recorded bag: {bag_out}  ({bag_size // 1024} KB)")
    return bag_out


# ---------------------------------------------------------------------------
# Bag analysis helpers
# ---------------------------------------------------------------------------


def _percentile(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    values.sort()
    idx = (len(values) - 1) * p / 100.0
    lo = int(idx)
    hi = lo + 1
    frac = idx - lo
    if hi >= len(values):
        return values[lo]
    return values[lo] * (1 - frac) + values[hi] * frac


def _stats(values: List[float], percentiles=(50, 95, 99)) -> Dict[str, float]:
    if not values:
        return {f"p{p}": 0.0 for p in percentiles}
    result = {f"p{p}": round(_percentile(values, p), 6) for p in percentiles}
    result["max"] = round(max(values), 6)
    return result


def _stats_no_p99(values: List[float]) -> Dict[str, float]:
    return _stats(values, percentiles=(50, 95))


# ---------------------------------------------------------------------------
# Vision backlog probe analysis
# ---------------------------------------------------------------------------


def analyse_vision_backlog(bag_path: str, mission: dict) -> Dict[str, Any]:
    """
    Analyse a combined bag for the vision_backlog_probe report suite.

    Metrics (replicates legacy vision_backlog_findings.json structure):
      frame_count_start_end, inference_time_ms, input_image_hz_assumed,
      publish_lag_ms, vision_output_hz, vision_processed_fps_from_frame_count

    New optional metrics (from throughput-opt-round-2 KVs):
      skipped_postprocess_newer_pending_delta, skip_ratio,
      fallback_ms_dist, tracker_ms_dist
    """
    import rosbag  # noqa – requires ROS env

    inference_us_list: List[float] = []
    publish_lag_ms_list: List[float] = []
    frame_counts: List[Tuple[float, int]] = []  # (bag_recv_sec, frame_count)
    skip_counts: List[int] = []
    fallback_ms_list: List[float] = []
    tracker_ms_list: List[float] = []

    det_recv_times: List[float] = []  # bag receive time for each detection msg

    def _kv(values_list, key) -> Optional[str]:
        for kv in values_list:
            if kv.key == key:
                return kv.value
        return None

    try:
        with rosbag.Bag(bag_path, "r") as bag:
            for topic, msg, t in bag.read_messages(
                topics=["/perception/vision/diagnostics", "/perception/vision/detections"]
            ):
                recv_sec = t.to_sec()

                if topic == "/perception/vision/diagnostics":
                    for status in msg.status:
                        if status.name == "vision_node":
                            v_inf = _kv(status.values, "inference_time_us")
                            if v_inf is not None:
                                try:
                                    inference_us_list.append(float(v_inf))
                                except ValueError:
                                    pass

                            v_fc = _kv(status.values, "frame_count")
                            if v_fc is not None:
                                try:
                                    frame_counts.append((recv_sec, int(v_fc)))
                                except ValueError:
                                    pass

                            v_skip = _kv(status.values, "skipped_postprocess_newer_pending")
                            if v_skip is not None:
                                try:
                                    skip_counts.append(int(v_skip))
                                except ValueError:
                                    pass

                            v_fb = _kv(status.values, "fallback_ms")
                            if v_fb is not None:
                                try:
                                    fallback_ms_list.append(float(v_fb))
                                except ValueError:
                                    pass

                            v_tr = _kv(status.values, "tracker_ms")
                            if v_tr is not None:
                                try:
                                    tracker_ms_list.append(float(v_tr))
                                except ValueError:
                                    pass

                elif topic == "/perception/vision/detections":
                    header_sec = msg.header.stamp.to_sec()
                    lag_ms = max((recv_sec - header_sec) * 1000.0, 0.0)
                    publish_lag_ms_list.append(lag_ms)
                    det_recv_times.append(recv_sec)

    except Exception as exc:
        print(f"  [error] rosbag read failed for {bag_path}: {exc}")
        return {}

    # inference_time_ms from inference_time_us
    infer_ms = [us / 1000.0 for us in inference_us_list]

    # Vision output Hz
    if len(det_recv_times) >= 2:
        duration = det_recv_times[-1] - det_recv_times[0]
        vision_output_hz = len(det_recv_times) / duration if duration > 0 else 0.0
    else:
        vision_output_hz = 0.0

    # vision_processed_fps from frame_count progression
    processed_fps_list: List[float] = []
    for i in range(1, len(frame_counts)):
        dt = frame_counts[i][0] - frame_counts[i - 1][0]
        dfc = frame_counts[i][1] - frame_counts[i - 1][1]
        if dt > 0 and dfc >= 0:
            processed_fps_list.append(dfc / dt)

    # frame_count_start_end
    fc_start = frame_counts[0][1] if frame_counts else 0
    fc_end = frame_counts[-1][1] if frame_counts else 0

    # skip ratio from newest-pending counter
    skip_delta = 0
    if len(skip_counts) >= 2:
        skip_delta = skip_counts[-1] - skip_counts[0]
    total_processed = max(fc_end - fc_start, 1)
    skip_ratio = round(skip_delta / total_processed, 4) if total_processed > 0 else 0.0

    result: Dict[str, Any] = {
        "frame_count_start_end": [fc_start, fc_end],
        "inference_time_ms": _stats(infer_ms),
        "input_image_hz_assumed": mission["input_image_hz_assumed"],
        "publish_lag_ms": _stats(publish_lag_ms_list),
        "vision_output_hz": round(vision_output_hz, 6),
        "vision_processed_fps_from_frame_count": _stats(processed_fps_list),
    }

    # Add new throughput-opt metrics if present in the bag
    if skip_delta > 0 or skip_counts:
        result["skipped_postprocess_newer_pending_delta"] = skip_delta
        result["skip_ratio"] = skip_ratio

    if [v for v in fallback_ms_list if v > 0]:
        nonzero_fb = [v for v in fallback_ms_list if v > 0]
        result["fallback_ms_dist"] = _stats(nonzero_fb, percentiles=(50, 95))
    else:
        result["fallback_ms_zero_frames"] = len(fallback_ms_list)

    if tracker_ms_list:
        nonzero_tr = [v for v in tracker_ms_list if v > 0]
        if nonzero_tr:
            result["tracker_ms_dist"] = _stats(nonzero_tr, percentiles=(50, 95))
        else:
            result["tracker_ms_all_zero"] = True

    return result


# ---------------------------------------------------------------------------
# Legacy timing review analysis
# ---------------------------------------------------------------------------


def analyse_timing_review(bag_path: str, mission: dict) -> Dict[str, Any]:
    """
    Analyse a combined bag for the legacy_timing_review report suite.

    Replicates legacy timing_review_findings.json structure exactly.
    """
    import rosbag  # noqa

    # ---- data accumulators ----
    trace_msgs: List[Tuple[float, Dict[str, str]]] = []  # (recv_sec, parsed_fields)
    vision_det_recv: List[float] = []  # bag recv times
    vision_det_header: List[float] = []  # header stamps
    fusion_det_count = 0
    latest_fusion_diag: Dict[str, str] = {}  # last set of fusion_* KVs
    fusion_sync_slop_sec: float = mission["budget_sec"]

    def _parse_trace(s: str) -> Dict[str, str]:
        fields: Dict[str, str] = {}
        for part in s.split():
            if "=" in part:
                k, _, v = part.partition("=")
                fields[k] = v
        return fields

    def _kv(values_list, key) -> Optional[str]:
        for kv in values_list:
            if kv.key == key:
                return kv.value
        return None

    try:
        with rosbag.Bag(bag_path, "r") as bag:
            for topic, msg, t in bag.read_messages(
                topics=[
                    "/perception/decision/trace",
                    "/perception/diagnostics",
                    "/perception/fusion/detections",
                    "/perception/vision/detections",
                ]
            ):
                recv_sec = t.to_sec()

                if topic == "/perception/decision/trace":
                    fields = _parse_trace(msg.data)
                    trace_msgs.append((recv_sec, fields))

                elif topic == "/perception/diagnostics":
                    for status in msg.status:
                        tmp: Dict[str, str] = {}
                        for kv in status.values:
                            if kv.key.startswith("fusion_"):
                                tmp[kv.key] = kv.value
                        if tmp:
                            latest_fusion_diag.update(tmp)
                            # Grab slop from diagnostics if available
                            if "fusion_sync_slop_sec" in tmp:
                                try:
                                    fusion_sync_slop_sec = float(tmp["fusion_sync_slop_sec"])
                                except ValueError:
                                    pass

                elif topic == "/perception/fusion/detections":
                    fusion_det_count += 1

                elif topic == "/perception/vision/detections":
                    vision_det_recv.append(recv_sec)
                    vision_det_header.append(msg.header.stamp.to_sec())

    except Exception as exc:
        print(f"  [error] rosbag read failed for {bag_path}: {exc}")
        return {}

    # ---- decision_trace stats ----
    n_frames = len(trace_msgs)
    publish_modes: Dict[str, int] = {}
    reason_counts: Dict[str, int] = {}
    wait_ms_list: List[float] = []

    for _, fields in trace_msgs:
        pm = fields.get("publish_mode", "unknown")
        publish_modes[pm] = publish_modes.get(pm, 0) + 1
        r = fields.get("reason", "")
        if r:
            reason_counts[r] = reason_counts.get(r, 0) + 1
        wm = fields.get("wait_ms")
        if wm is not None:
            try:
                wait_ms_list.append(float(wm))
            except ValueError:
                pass

    no_fused_count = publish_modes.get("raw_fallback", 0) + reason_counts.get("no_fused", 0)
    # Deduplicate: reason_counts["no_fused"] is already a subcount
    no_fused_ratio = (reason_counts.get("no_fused", 0) / n_frames) if n_frames > 0 else 0.0

    decision_trace = {
        "frames": n_frames,
        "no_fused_ratio": round(no_fused_ratio, 6),
        "publish_modes": publish_modes,
        "reason_counts": reason_counts,
        "wait_ms": _stats(wait_ms_list),
    }

    # ---- vision publish lag ----
    publish_lag_ms = [
        max((r - h) * 1000.0, 0.0) for r, h in zip(vision_det_recv, vision_det_header)
    ]

    # Vision output Hz
    if len(vision_det_recv) >= 2:
        duration = vision_det_recv[-1] - vision_det_recv[0]
        vision_publish_hz = round(len(vision_det_recv) / duration, 3) if duration > 0 else 0.0
    else:
        vision_publish_hz = 0.0

    # Vision publish interval
    vis_intervals_ms = [
        (vision_det_recv[i] - vision_det_recv[i - 1]) * 1000.0
        for i in range(1, len(vision_det_recv))
    ]

    # ---- vision_header_to_raw_ms (delta: raw_stamp - vision_header_stamp) ----
    # Extract raw LiDAR frame stamps from trace messages
    raw_stamps_sec: List[float] = []
    for _, fields in trace_msgs:
        sns = fields.get("stamp_ns")
        if sns:
            try:
                raw_stamps_sec.append(int(sns) / 1e9)
            except ValueError:
                pass

    header_to_raw_ms_list: List[float] = []
    raw_matched_set: set = set()

    for raw_idx, rs in enumerate(raw_stamps_sec):
        for vh in vision_det_header:
            delta_sec = rs - vh
            if abs(delta_sec) <= fusion_sync_slop_sec:
                header_to_raw_ms_list.append(delta_sec * 1000.0)
                raw_matched_set.add(raw_idx)

    raw_frames_with_match = len(raw_matched_set)
    raw_frames_total = len(raw_stamps_sec) if raw_stamps_sec else n_frames
    match_ratio = (
        round(raw_frames_with_match / raw_frames_total, 6) if raw_frames_total > 0 else 0.0
    )

    result: Dict[str, Any] = {
        "budget_sec": mission["budget_sec"],
        "decision_trace": decision_trace,
        "fusion_diagnostics": latest_fusion_diag,
        "fusion_topic_msg_count_in_record": fusion_det_count,
        "raw_frames_in_record_window": n_frames,
        "raw_frames_with_header_match_within_budget": raw_frames_with_match,
        "raw_frames_with_header_match_within_budget_ratio": match_ratio,
        "vision_header_to_raw_ms": _stats_no_p99(header_to_raw_ms_list),
        "vision_publish_hz": vision_publish_hz,
        "vision_publish_interval_ms": {
            "count": len(vision_det_recv),
            **_stats_no_p99(vis_intervals_ms),
        },
        "vision_publish_lag_ms": _stats(publish_lag_ms),
    }
    return result


# ---------------------------------------------------------------------------
# Markdown report generators
# ---------------------------------------------------------------------------


def _fmt_ms(d: Optional[Dict[str, float]]) -> str:
    if not d:
        return "N/A"
    parts = []
    for k in ("p50", "p95", "p99", "max"):
        if k in d:
            parts.append(f"{k}={d[k]:.1f}ms")
    return "  ".join(parts)


def build_backlog_md(data: Dict[str, Any], tag: str) -> str:
    lines = [
        f"# Vision Backlog Probe – {tag}\n",
        "_Generated by scripts/run_replay_reports.py_\n",
        "Metrics match the `vision_backlog_postfix` baseline. "
        "New rows (marked **NEW**) come from throughput-opt-round-2 KVs.\n",
    ]
    baseline_lag = {"trackdrive": 470, "skidpad": 682}
    baseline_infer = {"trackdrive": 429, "skidpad": 442}

    for mission_name, m in sorted(data.items()):
        lines.append(f"\n## {mission_name}\n")
        fc = m.get("frame_count_start_end", [0, 0])
        lines.append(f"- Frames processed: {fc[0]} → {fc[1]}  (delta={fc[1]-fc[0]})")
        lines.append(f"- Vision output Hz : {m.get('vision_output_hz', 0):.2f}")
        lines.append(f"- Input image Hz (bag): {m.get('input_image_hz_assumed', '?')}")

        lag = m.get("publish_lag_ms", {})
        b_lag = baseline_lag.get(mission_name, 0)
        delta_lag = lag.get("p50", 0) - b_lag
        sign = "+" if delta_lag >= 0 else ""
        lines.append(
            f"- Publish lag ms   : {_fmt_ms(lag)}"
            f"  ← baseline p50={b_lag}ms  ({sign}{delta_lag:.0f}ms)"
        )

        infer = m.get("inference_time_ms", {})
        b_inf = baseline_infer.get(mission_name, 0)
        delta_inf = infer.get("p50", 0) - b_inf
        sign2 = "+" if delta_inf >= 0 else ""
        lines.append(
            f"- Inference ms     : {_fmt_ms(infer)}"
            f"  ← baseline p50={b_inf}ms  ({sign2}{delta_inf:.0f}ms)"
        )

        fps = m.get("vision_processed_fps_from_frame_count", {})
        lines.append(
            f"- Processed FPS    : p50={fps.get('p50',0):.2f}  p95={fps.get('p95',0):.2f}  max={fps.get('max',0):.2f}"
        )

        # New metrics
        if "skipped_postprocess_newer_pending_delta" in m:
            lines.append(
                f"- **NEW** Skip newer-pending : delta={m['skipped_postprocess_newer_pending_delta']}"
                f"  ratio={m.get('skip_ratio', 0):.2%}"
            )
        if "fallback_ms_dist" in m:
            lines.append(f"- **NEW** Fallback ms (nonzero): {_fmt_ms(m['fallback_ms_dist'])}")
        elif "fallback_ms_zero_frames" in m:
            lines.append(
                f"- **NEW** Fallback ms: 0 in all {m['fallback_ms_zero_frames']} frames (✓ optimised out)"
            )
        if "tracker_ms_dist" in m:
            lines.append(f"- **NEW** Tracker ms (nonzero) : {_fmt_ms(m['tracker_ms_dist'])}")
        elif m.get("tracker_ms_all_zero"):
            lines.append("- **NEW** Tracker ms: 0 in all frames (skipped by newer-pending)")

    return "\n".join(lines) + "\n"


def build_timing_md(data: Dict[str, Any], tag: str) -> str:
    lines = [
        f"# Legacy Timing Review – {tag}\n",
        "_Generated by scripts/run_replay_reports.py_\n",
    ]
    baseline_no_fused = 1.0
    baseline_lag = {"trackdrive": 466, "autocross": 489, "skidpad": 682}

    for mission_name in ("trackdrive", "autocross", "skidpad"):
        m = data.get(mission_name)
        if not m:
            continue
        lines.append(f"\n## {mission_name}  (budget={m.get('budget_sec',0):.2f}s)\n")

        dt = m.get("decision_trace", {})
        no_fused_r = dt.get("no_fused_ratio", 1.0)
        delta_nf = no_fused_r - baseline_no_fused
        sign = "+" if delta_nf >= 0 else ""
        lines.append(f"- Decision frames  : {dt.get('frames', 0)}")
        lines.append(
            f"- no_fused ratio   : {no_fused_r:.2%}"
            f"  ← baseline={baseline_no_fused:.0%}  ({sign}{delta_nf:.2%})"
        )
        lines.append(f"- Publish modes    : {dt.get('publish_modes', {})}")
        lines.append(f"- Reason counts    : {dt.get('reason_counts', {})}")
        lines.append(f"- Wait ms          : {_fmt_ms(dt.get('wait_ms',{}))}")

        lag = m.get("vision_publish_lag_ms", {})
        b_lag = baseline_lag.get(mission_name, 0)
        delta_lag = lag.get("p50", 0) - b_lag
        sign2 = "+" if delta_lag >= 0 else ""
        lines.append(
            f"- Vision lag ms    : {_fmt_ms(lag)}"
            f"  ← baseline p50={b_lag}ms  ({sign2}{delta_lag:.0f}ms)"
        )
        lines.append(f"- Vision Hz        : {m.get('vision_publish_hz', 0):.2f}")
        lines.append(f"- header→raw ms    : {_fmt_ms(m.get('vision_header_to_raw_ms',{}))}")
        lines.append(
            f"- raw match/budget : {m.get('raw_frames_with_header_match_within_budget',0)}"
            f" / {m.get('raw_frames_in_record_window',0)}"
            f"  ({m.get('raw_frames_with_header_match_within_budget_ratio',0):.1%})"
        )

        fd = m.get("fusion_diagnostics", {})
        if fd:
            lines.append(
                f"- Fusion totals    : pair_total={fd.get('fusion_pair_total','?')}"
                f"  pair_used={fd.get('fusion_pair_used','?')}"
                f"  published={fd.get('fusion_messages_published','?')}"
                f"  frames_no_sync={fd.get('fusion_frames_no_sync','?')}"
            )

    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# Main orchestration
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="Run replay report suites")
    parser.add_argument(
        "--outdir",
        default=os.path.join(os.path.dirname(__file__), "..", "perf_reports", "data"),
        help="Base output directory for report artefacts",
    )
    parser.add_argument(
        "--skip-replay",
        action="store_true",
        help="Skip the roslaunch replay and re-analyse existing bags",
    )
    parser.add_argument(
        "--bags-dir",
        default=None,
        help="When --skip-replay is set, look for combined bags inside this directory",
    )
    args = parser.parse_args()

    tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir_base = os.path.realpath(args.outdir)

    probe_outdir = os.path.join(outdir_base, f"vision_backlog_probe_opt2_{tag}")
    timing_outdir = os.path.join(outdir_base, f"legacy_timing_review_opt2_{tag}")
    os.makedirs(probe_outdir, exist_ok=True)
    os.makedirs(timing_outdir, exist_ok=True)

    print(f"\n=== Replay Report Runner – tag={tag} ===")
    print(f"  probe  → {probe_outdir}")
    print(f"  timing → {timing_outdir}")

    # Map mission name → combined bag path
    bags: Dict[str, str] = {}

    if args.skip_replay:
        bags_dir = args.bags_dir or probe_outdir
        for m in MISSIONS:
            candidate = os.path.join(bags_dir, m["name"], f"{m['name']}_combined.bag")
            if os.path.exists(candidate):
                bags[m["name"]] = candidate
            else:
                print(f"  [warn] no existing bag for {m['name']} in {bags_dir}")
    else:
        print("\n--- Phase 1: rosbag replay ---")
        for mission in MISSIONS:
            mdir = os.path.join(probe_outdir, mission["name"])
            os.makedirs(mdir, exist_ok=True)
            print(f"\n[{mission['name']}] starting replay…")
            bag_path = run_mission_replay(mission, mdir)
            if bag_path:
                bags[mission["name"]] = bag_path
            else:
                print(f"  [skip] {mission['name']} replay failed – excluded from reports")

    # ---- Phase 2: analyse ----
    print("\n--- Phase 2: analysis ---")
    backlog_data: Dict[str, Any] = {}
    timing_data: Dict[str, Any] = {}

    for mission in MISSIONS:
        name = mission["name"]
        bag = bags.get(name)
        if not bag:
            print(f"  [skip] no bag for {name}")
            continue
        print(f"  analysing {name}…")
        if mission["in_backlog_probe"]:
            result = analyse_vision_backlog(bag, mission)
            if result:
                backlog_data[name] = result
        if mission["in_timing_review"]:
            result = analyse_timing_review(bag, mission)
            if result:
                timing_data[name] = result

    # ---- Phase 3: write reports ----
    print("\n--- Phase 3: writing reports ---")

    def _write(path, content):
        with open(path, "w") as f:
            f.write(content)
        print(f"  wrote {path}")

    _write(
        os.path.join(probe_outdir, "vision_backlog_findings.json"),
        json.dumps(backlog_data, indent=2),
    )
    _write(
        os.path.join(probe_outdir, "vision_backlog_findings.md"),
        build_backlog_md(backlog_data, tag),
    )
    _write(
        os.path.join(timing_outdir, "timing_review_findings.json"),
        json.dumps(timing_data, indent=2),
    )
    _write(
        os.path.join(timing_outdir, "timing_review_findings.md"),
        build_timing_md(timing_data, tag),
    )

    # ---- Phase 4: print summary to console ----
    print("\n=== Summary ===")
    for name, m in backlog_data.items():
        lag = m.get("publish_lag_ms", {})
        skip = m.get("skipped_postprocess_newer_pending_delta", 0)
        skip_ratio = m.get("skip_ratio", 0)
        print(
            f"  [backlog/{name}] publish_lag p50={lag.get('p50',0):.0f}ms"
            f"  p95={lag.get('p95',0):.0f}ms"
            f"  skip_newer_pending={skip} ({skip_ratio:.1%})"
        )
    for name, m in timing_data.items():
        lag = m.get("vision_publish_lag_ms", {})
        nr = m.get("decision_trace", {}).get("no_fused_ratio", 1.0)
        fps = m.get("decision_trace", {}).get("frames", 0)
        print(
            f"  [timing/{name}] publish_lag p50={lag.get('p50',0):.0f}ms"
            f"  no_fused={nr:.0%}  frames={fps}"
        )

    print(f"\nDone.  Artefacts in:\n  {probe_outdir}\n  {timing_outdir}")


if __name__ == "__main__":
    main()
