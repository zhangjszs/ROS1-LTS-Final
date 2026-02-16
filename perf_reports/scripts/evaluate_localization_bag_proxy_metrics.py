#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import rosbag


def _norm_topic(topic: str) -> str:
    return topic.lstrip("/")


def _same_topic(a: str, b: str) -> bool:
    return _norm_topic(a) == _norm_topic(b)


def _msg_stamp(msg, bag_t_sec: float) -> float:
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        t = msg.header.stamp.to_sec()
        if t > 0.0:
            return t
    return bag_t_sec


def _as_float(raw) -> Optional[float]:
    try:
        return float(raw)
    except Exception:
        return None


def _as_bool(raw) -> Optional[bool]:
    text = str(raw).strip().lower()
    if text in {"1", "true", "t", "yes", "y"}:
        return True
    if text in {"0", "false", "f", "no", "n"}:
        return False
    return None


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def _rate_hz(times: List[float]) -> Optional[float]:
    if len(times) < 2:
        return None
    dt = times[-1] - times[0]
    if dt <= 1e-6:
        return None
    return float(len(times) - 1) / dt


def _mean(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return sum(values) / len(values)


def _quantile(values: List[float], q: float) -> Optional[float]:
    if not values:
        return None
    vals = sorted(values)
    idx = int(max(0, min(len(vals) - 1, round((len(vals) - 1) * q))))
    return vals[idx]


def _std(values: List[float]) -> Optional[float]:
    if len(values) < 2:
        return None
    mu = _mean(values)
    assert mu is not None
    var = sum((v - mu) ** 2 for v in values) / len(values)
    return math.sqrt(var)


def _format_num(v: Optional[float], ndigits: int = 4) -> str:
    if v is None:
        return "N/A"
    return f"{v:.{ndigits}f}"


def _extract_localization_diag_kv(msg) -> Dict[str, str]:
    for status in getattr(msg, "status", []):
        name = getattr(status, "name", "")
        if "localization_entry_health" not in name:
            continue
        kv = {}
        for pair in getattr(status, "values", []):
            kv[str(getattr(pair, "key", ""))] = str(getattr(pair, "value", ""))
        return kv
    return {}


def _compute_pose_jitter(car_samples: List[Tuple[float, float, float]]) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    if len(car_samples) < 3:
        return None, None, None

    pos_second_diff = []
    yaw_second_diff = []
    for i in range(1, len(car_samples) - 1):
        x0, y0, y0aw = car_samples[i - 1]
        x1, y1, y1aw = car_samples[i]
        x2, y2, y2aw = car_samples[i + 1]
        ddx = x2 - 2.0 * x1 + x0
        ddy = y2 - 2.0 * y1 + y0
        pos_second_diff.append(math.hypot(ddx, ddy))
        dyaw = _angle_diff(y2aw, y1aw) - _angle_diff(y1aw, y0aw)
        yaw_second_diff.append(abs(dyaw))

    closure_error = math.hypot(car_samples[-1][0] - car_samples[0][0], car_samples[-1][1] - car_samples[0][1])
    return _mean(pos_second_diff), math.degrees(_mean(yaw_second_diff) or 0.0), closure_error


def _compute_map_repeat_consistency(cone_history: Dict[int, List[Tuple[float, float]]]) -> Optional[float]:
    id_sigmas = []
    for cid, pts in cone_history.items():
        if cid <= 0 or len(pts) < 3:
            continue
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        d = [math.hypot(p[0] - mx, p[1] - my) for p in pts]
        s = _std(d)
        if s is not None:
            id_sigmas.append(s)
    return _mean(id_sigmas)


def _compute_recovery_metrics(state_series: List[Tuple[float, str]]) -> Tuple[Optional[float], Optional[float], Optional[float], int]:
    if not state_series:
        return None, None, None, 0

    lost_entries = 0
    recovered = 0
    latencies = []
    active_lost_t: Optional[float] = None
    last_state = "TRACKING"
    for t, state in state_series:
        if state != "TRACKING" and last_state == "TRACKING":
            lost_entries += 1
            active_lost_t = t
        elif state == "TRACKING" and last_state != "TRACKING" and active_lost_t is not None:
            recovered += 1
            latencies.append(max(0.0, t - active_lost_t))
            active_lost_t = None
        last_state = state

    success_rate = (float(recovered) / float(lost_entries)) if lost_entries > 0 else None
    return success_rate, _mean(latencies), _quantile(latencies, 0.95), lost_entries


def evaluate(args) -> Dict:
    required_topics = {
        "detections": args.detections_topic,
        "car_state": args.car_state_topic,
        "cone_map": args.cone_map_topic,
        "tf": args.tf_topic,
        "ins": args.ins_topic,
        "diagnostics": args.diagnostics_topic,
    }

    topic_info = {k: {"topic": v, "present": False, "count": 0} for k, v in required_topics.items()}
    detection_times: List[float] = []
    car_times: List[float] = []
    car_pose: List[Tuple[float, float, float]] = []
    cone_times: List[float] = []
    tf_times: List[float] = []
    ins_times: List[float] = []
    diag_times: List[float] = []
    cone_history: Dict[int, List[Tuple[float, float]]] = defaultdict(list)

    state_series: List[Tuple[float, str]] = []
    match_good_flags: List[bool] = []
    match_ratio_values: List[float] = []
    mean_match_distance_values: List[float] = []
    tf_lag_values: List[float] = []

    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, msg, bag_t in bag.read_messages():
            t = bag_t.to_sec()
            for name, cfg_topic in required_topics.items():
                if _same_topic(topic, cfg_topic):
                    topic_info[name]["present"] = True
                    topic_info[name]["count"] += 1

            if _same_topic(topic, args.detections_topic):
                detection_times.append(_msg_stamp(msg, t))
            elif _same_topic(topic, args.car_state_topic):
                ts = _msg_stamp(msg, t)
                car_times.append(ts)
                if hasattr(msg, "car_state"):
                    car_pose.append(
                        (
                            float(getattr(msg.car_state, "x", 0.0)),
                            float(getattr(msg.car_state, "y", 0.0)),
                            float(getattr(msg.car_state, "theta", 0.0)),
                        )
                    )
            elif _same_topic(topic, args.cone_map_topic):
                ts = _msg_stamp(msg, t)
                cone_times.append(ts)
                for cone in getattr(msg, "cone", []):
                    cid = int(getattr(cone, "id", 0))
                    gx = float(getattr(getattr(cone, "position_global", None), "x", 0.0))
                    gy = float(getattr(getattr(cone, "position_global", None), "y", 0.0))
                    cone_history[cid].append((gx, gy))
            elif _same_topic(topic, args.tf_topic):
                tf_times.append(_msg_stamp(msg, t))
            elif _same_topic(topic, args.ins_topic):
                ins_times.append(_msg_stamp(msg, t))
            elif _same_topic(topic, args.diagnostics_topic):
                ts = _msg_stamp(msg, t)
                diag_times.append(ts)
                kv = _extract_localization_diag_kv(msg)
                if not kv:
                    continue
                state = kv.get("mapper_state")
                if state:
                    state_series.append((ts, state))
                good = _as_bool(kv.get("cone_last_frame_good"))
                if good is not None:
                    match_good_flags.append(good)
                mr = _as_float(kv.get("cone_last_match_ratio"))
                if mr is not None:
                    match_ratio_values.append(mr)
                md = _as_float(kv.get("mapper_mean_match_distance"))
                if md is not None:
                    mean_match_distance_values.append(md)
                lag = _as_float(kv.get("tf_lag_sec"))
                if lag is not None:
                    tf_lag_values.append(lag)

    missing_topics = [k for k, v in topic_info.items() if not v["present"]]
    non_tracking_ratio = None
    if state_series:
        non_tracking = sum(1 for _, s in state_series if s != "TRACKING")
        non_tracking_ratio = non_tracking / float(len(state_series))

    match_success_rate = None
    if match_good_flags:
        match_success_rate = sum(1 for g in match_good_flags if g) / float(len(match_good_flags))

    reloc_success_rate, reloc_latency_mean_s, reloc_latency_p95_s, reloc_event_count = _compute_recovery_metrics(state_series)
    pos_jitter_m, heading_jitter_deg, closure_error_m = _compute_pose_jitter(car_pose)
    repeat_consistency_m = _compute_map_repeat_consistency(cone_history)

    payload = {
        "version": 1,
        "generated_at": datetime.now().isoformat(),
        "bag": str(args.bag),
        "contract": {
            "required_topics": topic_info,
            "missing_topics": missing_topics,
            "all_present": len(missing_topics) == 0,
        },
        "metrics": {
            "detections_rate_hz": _rate_hz(detection_times),
            "car_state_rate_hz": _rate_hz(car_times),
            "cone_map_rate_hz": _rate_hz(cone_times),
            "diagnostics_rate_hz": _rate_hz(diag_times),
            "tf_rate_hz": _rate_hz(tf_times),
            "ins_rate_hz": _rate_hz(ins_times),
            "match_success_rate": match_success_rate,
            "non_tracking_ratio": non_tracking_ratio,
            "relocalization_success_rate": reloc_success_rate,
            "relocalization_latency_mean_s": reloc_latency_mean_s,
            "relocalization_latency_p95_s": reloc_latency_p95_s,
            "relocalization_event_count": reloc_event_count,
            "position_jitter_m": pos_jitter_m,
            "heading_jitter_deg": heading_jitter_deg,
            "closure_error_m": closure_error_m,
            "map_repeat_consistency_m": repeat_consistency_m,
            "match_ratio_mean": _mean(match_ratio_values),
            "match_ratio_p05": _quantile(match_ratio_values, 0.05),
            "mean_match_distance_mean_m": _mean(mean_match_distance_values),
            "mean_match_distance_std_m": _std(mean_match_distance_values),
            "tf_lag_p95_s": _quantile(tf_lag_values, 0.95),
        },
    }
    return payload


def _write_markdown(path: Path, payload: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    m = payload["metrics"]
    c = payload["contract"]
    lines = [
        "# Localization Bag Proxy Metrics",
        "",
        f"- generated_at: `{payload['generated_at']}`",
        f"- bag: `{payload['bag']}`",
        f"- all_required_topics_present: `{c['all_present']}`",
        f"- missing_topics: `{', '.join(c['missing_topics']) if c['missing_topics'] else 'none'}`",
        "",
        "| metric | value |",
        "|---|---:|",
        f"| car_state_rate_hz | {_format_num(m['car_state_rate_hz'])} |",
        f"| cone_map_rate_hz | {_format_num(m['cone_map_rate_hz'])} |",
        f"| detections_rate_hz | {_format_num(m['detections_rate_hz'])} |",
        f"| diagnostics_rate_hz | {_format_num(m['diagnostics_rate_hz'])} |",
        f"| tf_rate_hz | {_format_num(m['tf_rate_hz'])} |",
        f"| ins_rate_hz | {_format_num(m['ins_rate_hz'])} |",
        f"| match_success_rate | {_format_num(m['match_success_rate'])} |",
        f"| non_tracking_ratio | {_format_num(m['non_tracking_ratio'])} |",
        f"| relocalization_success_rate | {_format_num(m['relocalization_success_rate'])} |",
        f"| relocalization_latency_mean_s | {_format_num(m['relocalization_latency_mean_s'])} |",
        f"| relocalization_latency_p95_s | {_format_num(m['relocalization_latency_p95_s'])} |",
        f"| position_jitter_m | {_format_num(m['position_jitter_m'])} |",
        f"| heading_jitter_deg | {_format_num(m['heading_jitter_deg'])} |",
        f"| closure_error_m | {_format_num(m['closure_error_m'])} |",
        f"| map_repeat_consistency_m | {_format_num(m['map_repeat_consistency_m'])} |",
        f"| match_ratio_mean | {_format_num(m['match_ratio_mean'])} |",
        f"| match_ratio_p05 | {_format_num(m['match_ratio_p05'])} |",
        f"| mean_match_distance_mean_m | {_format_num(m['mean_match_distance_mean_m'])} |",
        f"| mean_match_distance_std_m | {_format_num(m['mean_match_distance_std_m'])} |",
        f"| tf_lag_p95_s | {_format_num(m['tf_lag_p95_s'])} |",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Evaluate localization proxy metrics and topic contract directly from rosbag."
    )
    parser.add_argument("--bag", type=Path, required=True, help="Input rosbag path.")
    parser.add_argument("--detections-topic", type=str, default="perception/lidar_cluster/detections")
    parser.add_argument("--car-state-topic", type=str, default="localization/car_state")
    parser.add_argument("--cone-map-topic", type=str, default="localization/cone_map")
    parser.add_argument("--tf-topic", type=str, default="/tf")
    parser.add_argument("--ins-topic", type=str, default="sensors/ins")
    parser.add_argument("--diagnostics-topic", type=str, default="localization/diagnostics")
    parser.add_argument(
        "--output-json",
        type=Path,
        default=Path("docs/baseline/localization_proxy_metrics.json"),
    )
    parser.add_argument(
        "--output-md",
        type=Path,
        default=Path("docs/baseline/localization_proxy_metrics.md"),
    )
    args = parser.parse_args()

    payload = evaluate(args)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    _write_markdown(args.output_md, payload)

    print(f"[OK] proxy metrics json: {args.output_json}")
    print(f"[OK] proxy metrics markdown: {args.output_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
