#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import glob
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml


DEFAULT_METRICS = ["t_pass_ms", "t_ground_ms", "t_cluster_ms", "t_total_ms", "bytes"]


def _load_yaml(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data or {}


def _load_json(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _resolve_metrics(gate_config: Path) -> List[str]:
    if not gate_config.exists():
        return DEFAULT_METRICS
    data = _load_yaml(gate_config)
    metrics = data.get("gate", {}).get("perf", {}).get("metrics", [])
    if not metrics:
        return DEFAULT_METRICS
    return [str(item) for item in metrics]


def _is_valid_entry(entry: Dict, required_metrics: List[str]) -> bool:
    metrics = entry.get("metrics", {})
    return all(metric in metrics for metric in required_metrics)


def _select_perf_entry(
    perf_data_file: Optional[Path],
    perf_data_dir: Path,
    node_name: str,
    required_metrics: List[str],
) -> Tuple[Path, Dict, Dict]:
    if perf_data_file:
        data = _load_json(perf_data_file)
        entries = data.get("nodes", {}).get(node_name, [])
        if not entries:
            raise RuntimeError(f"Node '{node_name}' not found in {perf_data_file}")
        latest = entries[-1]
        if not _is_valid_entry(latest, required_metrics):
            raise RuntimeError(
                f"Missing required metrics in '{perf_data_file}': {required_metrics}"
            )
        return perf_data_file, data, latest

    pattern = str(perf_data_dir / "perf_data_*.json")
    candidates = sorted(glob.glob(pattern), reverse=True)
    for candidate in candidates:
        path = Path(candidate)
        try:
            data = _load_json(path)
        except Exception:
            continue
        entries = data.get("nodes", {}).get(node_name, [])
        if not entries:
            continue
        latest = entries[-1]
        if _is_valid_entry(latest, required_metrics):
            return path, data, latest

    raise RuntimeError(
        f"No valid perf data found in {perf_data_dir} for node '{node_name}' "
        f"with metrics {required_metrics}"
    )


def _build_output_payload(
    source_file: Path,
    data: Dict,
    latest_entry: Dict,
    required_metrics: List[str],
) -> Dict:
    selected_metrics = {}
    for metric in required_metrics:
        selected_metrics[metric] = latest_entry["metrics"][metric]

    return {
        "version": 1,
        "generated_at": datetime.now().isoformat(),
        "source_perf_file": str(source_file),
        "collection_time": data.get("collection_time", ""),
        "scenario": data.get("scenario", ""),
        "note": data.get("note", ""),
        "git_info": data.get("git_info", {}),
        "node": latest_entry.get("node", ""),
        "window_size": latest_entry.get("window_size", 0),
        "baseline_metrics": selected_metrics,
    }


def _write_markdown(
    output_md: Path, payload: Dict, required_metrics: List[str], manifest: Optional[Dict]
) -> None:
    output_md.parent.mkdir(parents=True, exist_ok=True)
    commit = (
        manifest.get("baseline", {}).get("git", {}).get("commit", "")
        if manifest
        else ""
    )
    bag_path = (
        manifest.get("baseline", {}).get("dataset", {}).get("bag_path", "")
        if manifest
        else ""
    )

    lines = []
    lines.append("# Baseline Performance Dashboard")
    lines.append("")
    lines.append(f"- generated_at: `{payload['generated_at']}`")
    lines.append(f"- source_perf_file: `{payload['source_perf_file']}`")
    lines.append(f"- node: `{payload['node']}`")
    lines.append(f"- window_size: `{payload['window_size']}`")
    if commit:
        lines.append(f"- baseline_commit: `{commit}`")
    if bag_path:
        lines.append(f"- bag_path: `{bag_path}`")
    lines.append("")
    lines.append("| metric | mean | p50 | p95 | p99 | max |")
    lines.append("|---|---:|---:|---:|---:|---:|")

    for metric in required_metrics:
        stats = payload["baseline_metrics"][metric]
        lines.append(
            "| {metric} | {mean:.3f} | {p50:.3f} | {p95:.3f} | {p99:.3f} | {maxv:.3f} |".format(
                metric=metric,
                mean=float(stats.get("mean", 0.0)),
                p50=float(stats.get("p50", 0.0)),
                p95=float(stats.get("p95", 0.0)),
                p99=float(stats.get("p99", 0.0)),
                maxv=float(stats.get("max", 0.0)),
            )
        )

    output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate baseline performance dashboard and json snapshot."
    )
    parser.add_argument(
        "--perf-data-file",
        type=Path,
        default=None,
        help="Specific perf_data_*.json file. If omitted, auto-pick latest valid.",
    )
    parser.add_argument(
        "--perf-data-dir",
        type=Path,
        default=Path("perf_reports/data"),
        help="Directory containing perf_data_*.json files.",
    )
    parser.add_argument(
        "--node", type=str, default="lidar_cluster", help="Target node name."
    )
    parser.add_argument(
        "--gate-config",
        type=Path,
        default=Path("docs/baseline/rollback_gate.yaml"),
        help="Gate config used to resolve required perf metrics.",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("docs/baseline/baseline_manifest.yaml"),
        help="Optional baseline manifest yaml.",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=Path("docs/baseline/performance_baseline.json"),
        help="Output baseline perf snapshot json.",
    )
    parser.add_argument(
        "--output-md",
        type=Path,
        default=Path("docs/baseline/performance_dashboard.md"),
        help="Output markdown dashboard path.",
    )
    args = parser.parse_args()

    required_metrics = _resolve_metrics(args.gate_config)
    source_file, data, latest_entry = _select_perf_entry(
        args.perf_data_file, args.perf_data_dir, args.node, required_metrics
    )

    payload = _build_output_payload(source_file, data, latest_entry, required_metrics)
    manifest = _load_yaml(args.manifest) if args.manifest.exists() else None
    if manifest:
        payload["manifest_git"] = manifest.get("baseline", {}).get("git", {})
        payload["manifest_dataset"] = manifest.get("baseline", {}).get("dataset", {})

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    _write_markdown(args.output_md, payload, required_metrics, manifest)

    print(f"[OK] perf baseline json: {args.output_json}")
    print(f"[OK] perf baseline markdown: {args.output_md}")
    print(f"[OK] source perf data: {source_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
