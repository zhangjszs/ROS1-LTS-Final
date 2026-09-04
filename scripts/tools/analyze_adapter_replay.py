#!/usr/bin/env python3
import argparse
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import rosbag

TRACE_PATTERN = re.compile(
    r"stamp_ns=(?P<stamp_ns>\d+)\s+publish_mode=(?P<publish_mode>\S+)\s+reason=(?P<reason>\S+)\s+wait_ms=(?P<wait_ms>[-+]?\d+(?:\.\d+)?)"
)


def percentile(values: List[float], pct: float) -> Optional[float]:
    if not values:
        return None
    sorted_values = sorted(values)
    if len(sorted_values) == 1:
        return sorted_values[0]
    rank = (len(sorted_values) - 1) * pct / 100.0
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return sorted_values[low]
    fraction = rank - low
    return sorted_values[low] + (sorted_values[high] - sorted_values[low]) * fraction


def parse_trace(trace_line: str) -> Optional[Dict[str, object]]:
    match = TRACE_PATTERN.search(trace_line.strip())
    if not match:
        return None
    return {
        "stamp_ns": int(match.group("stamp_ns")),
        "publish_mode": match.group("publish_mode"),
        "reason": match.group("reason"),
        "wait_ms": float(match.group("wait_ms")),
    }


def analyze_bag(bag_path: Path, scenario: str) -> Dict[str, object]:
    traces: List[Dict[str, object]] = []
    output_times_ms: List[float] = []

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, bag_time in bag.read_messages():
            if topic == "/perception/decision/trace":
                trace = parse_trace(msg.data)
                if trace is not None:
                    traces.append(trace)
            elif topic == "/perception/decision/detections":
                output_times_ms.append(bag_time.to_sec() * 1000.0)

    reasons = Counter(trace["reason"] for trace in traces)
    modes = Counter(trace["publish_mode"] for trace in traces)
    waits = [trace["wait_ms"] for trace in traces]
    gaps = [curr - prev for prev, curr in zip(output_times_ms, output_times_ms[1:])]
    frames = len(traces)

    return {
        "scenario": scenario,
        "bag": str(bag_path),
        "frames": frames,
        "fused_frames": modes.get("fused", 0),
        "raw_fallback_frames": modes.get("raw_fallback", 0),
        "fused_ratio": (modes.get("fused", 0) / frames) if frames else 0.0,
        "raw_fallback_ratio": (modes.get("raw_fallback", 0) / frames) if frames else 0.0,
        "reason_counts": dict(sorted(reasons.items())),
        "wait_ms": {
            "p50": percentile(waits, 50.0),
            "p95": percentile(waits, 95.0),
            "p99": percentile(waits, 99.0),
            "max": max(waits) if waits else None,
        },
        "output_gap_ms": {
            "p99": percentile(gaps, 99.0),
            "max": max(gaps) if gaps else None,
        },
    }


def write_summary(summary: List[Dict[str, object]], out_dir: Path) -> None:
    json_path = out_dir / "summary.json"
    csv_path = out_dir / "summary.csv"
    md_path = out_dir / "summary.md"

    json_path.write_text(json.dumps(summary, indent=2, sort_keys=True))

    with csv_path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "scenario",
                "frames",
                "fused_frames",
                "raw_fallback_frames",
                "fused_ratio",
                "raw_fallback_ratio",
                "reason_counts",
                "wait_p50_ms",
                "wait_p95_ms",
                "wait_p99_ms",
                "wait_max_ms",
                "gap_p99_ms",
                "gap_max_ms",
            ]
        )
        for row in summary:
            writer.writerow(
                [
                    row["scenario"],
                    row["frames"],
                    row["fused_frames"],
                    row["raw_fallback_frames"],
                    f"{row['fused_ratio']:.3f}",
                    f"{row['raw_fallback_ratio']:.3f}",
                    json.dumps(row["reason_counts"], sort_keys=True),
                    row["wait_ms"]["p50"],
                    row["wait_ms"]["p95"],
                    row["wait_ms"]["p99"],
                    row["wait_ms"]["max"],
                    row["output_gap_ms"]["p99"],
                    row["output_gap_ms"]["max"],
                ]
            )

    lines = [
        "# Adapter Replay Summary",
        "",
        "| Scenario | Frames | Fused | Raw Fallback | Reason Counts | wait_ms p50/p95/p99/max | output_gap_ms p99/max |",
        "| --- | ---: | ---: | ---: | --- | --- | --- |",
    ]
    for row in summary:
        wait = row["wait_ms"]
        gap = row["output_gap_ms"]
        lines.append(
            "| {scenario} | {frames} | {fused_frames} ({fused_ratio:.1%}) | {raw_fallback_frames} ({raw_fallback_ratio:.1%}) | `{reason_counts}` | `{p50:.1f}/{p95:.1f}/{p99:.1f}/{maxv:.1f}` | `{gp99:.1f}/{gmax:.1f}` |".format(
                scenario=row["scenario"],
                frames=row["frames"],
                fused_frames=row["fused_frames"],
                fused_ratio=row["fused_ratio"],
                raw_fallback_frames=row["raw_fallback_frames"],
                raw_fallback_ratio=row["raw_fallback_ratio"],
                reason_counts=json.dumps(row["reason_counts"], sort_keys=True),
                p50=wait["p50"] or 0.0,
                p95=wait["p95"] or 0.0,
                p99=wait["p99"] or 0.0,
                maxv=wait["max"] or 0.0,
                gp99=gap["p99"] or 0.0,
                gmax=gap["max"] or 0.0,
            )
        )
    md_path.write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze adapter replay result bags.")
    parser.add_argument("bags", nargs="+", help="Bag paths to analyze")
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    summary = []
    for bag_arg in args.bags:
        bag_path = Path(bag_arg)
        scenario = bag_path.stem.replace("_results", "")
        metrics = analyze_bag(bag_path, scenario)
        summary.append(metrics)
        (out_dir / f"{scenario}_summary.json").write_text(
            json.dumps(metrics, indent=2, sort_keys=True)
        )

    write_summary(summary, out_dir)


if __name__ == "__main__":
    main()
