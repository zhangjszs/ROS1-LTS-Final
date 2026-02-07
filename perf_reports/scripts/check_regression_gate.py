#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml


def _load_yaml(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data or {}


def _load_json(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _extract_perf_metric_map(data: Dict, node_name: str) -> Dict:
    if "baseline_metrics" in data:
        return data["baseline_metrics"]

    if "nodes" in data:
        entries = data.get("nodes", {}).get(node_name, [])
        if not entries:
            return {}
        return entries[-1].get("metrics", {})

    return {}


def _extract_local_metric_map(data: Dict) -> Dict:
    if "metrics" in data:
        return data["metrics"]
    return {}


def _metric_value(metric_map: Dict, metric_name: str) -> Optional[float]:
    raw = metric_map.get(metric_name)
    if raw is None:
        return None
    if isinstance(raw, (int, float)):
        return float(raw)
    if isinstance(raw, dict):
        if "mean" in raw:
            return float(raw["mean"])
        if "value" in raw:
            return float(raw["value"])
    return None


def _compute_regression_pct(baseline: float, candidate: float) -> float:
    if baseline == 0.0:
        if candidate == 0.0:
            return 0.0
        return math.inf
    return ((candidate - baseline) / baseline) * 100.0


def _check_metric(
    metric_name: str,
    baseline_value: Optional[float],
    candidate_value: Optional[float],
    threshold_pct: float,
    category: str,
) -> Dict:
    result = {
        "metric": metric_name,
        "category": category,
        "baseline": baseline_value,
        "candidate": candidate_value,
        "threshold_pct": threshold_pct,
        "regression_pct": None,
        "status": "skipped",
        "reason": "",
    }

    if baseline_value is None or candidate_value is None:
        result["reason"] = "missing_value"
        return result

    regression_pct = _compute_regression_pct(baseline_value, candidate_value)
    result["regression_pct"] = regression_pct

    if regression_pct > threshold_pct:
        result["status"] = "fail"
        result["reason"] = "threshold_exceeded"
    else:
        result["status"] = "pass"
        result["reason"] = "within_threshold"

    return result


def _is_latency_like_metric(metric_name: str) -> bool:
    key = metric_name.lower()
    return ("time" in key) or ("latency" in key) or key.endswith("_ms")


def _write_markdown(output_md: Path, payload: Dict) -> None:
    output_md.parent.mkdir(parents=True, exist_ok=True)
    lines: List[str] = []
    lines.append("# Regression Gate Result")
    lines.append("")
    lines.append(f"- generated_at: `{payload['generated_at']}`")
    lines.append(f"- overall_status: `{payload['overall_status']}`")
    lines.append(f"- fail_count: `{payload['summary']['fail_count']}`")
    lines.append(f"- pass_count: `{payload['summary']['pass_count']}`")
    lines.append(f"- skipped_count: `{payload['summary']['skipped_count']}`")
    lines.append("")
    lines.append("| metric | category | baseline | candidate | regression(%) | threshold(%) | status |")
    lines.append("|---|---|---:|---:|---:|---:|---|")

    for item in payload["results"]:
        baseline = "N/A" if item["baseline"] is None else f"{item['baseline']:.6f}"
        candidate = "N/A" if item["candidate"] is None else f"{item['candidate']:.6f}"
        regression = (
            "N/A"
            if item["regression_pct"] is None
            else ("INF" if math.isinf(item["regression_pct"]) else f"{item['regression_pct']:.3f}")
        )
        lines.append(
            f"| {item['metric']} | {item['category']} | {baseline} | {candidate} | "
            f"{regression} | {item['threshold_pct']:.3f} | {item['status']} |"
        )

    output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check rollback regression gate.")
    parser.add_argument(
        "--gate-config",
        type=Path,
        default=Path("docs/baseline/rollback_gate.yaml"),
        help="Rollback gate yaml path.",
    )
    parser.add_argument(
        "--perf-baseline",
        type=Path,
        default=Path("docs/baseline/performance_baseline.json"),
        help="Baseline perf metrics json.",
    )
    parser.add_argument(
        "--perf-candidate",
        type=Path,
        required=True,
        help="Candidate perf metrics json.",
    )
    parser.add_argument(
        "--loc-baseline",
        type=Path,
        default=Path("docs/baseline/localization_baseline.json"),
        help="Baseline localization metrics json.",
    )
    parser.add_argument(
        "--loc-candidate",
        type=Path,
        default=None,
        help="Candidate localization metrics json.",
    )
    parser.add_argument(
        "--node", type=str, default="lidar_cluster", help="Perf node name."
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=Path("docs/baseline/regression_gate_result.json"),
        help="Output gate result json.",
    )
    parser.add_argument(
        "--output-md",
        type=Path,
        default=Path("docs/baseline/regression_gate_result.md"),
        help="Output gate result markdown.",
    )
    args = parser.parse_args()

    gate = _load_yaml(args.gate_config).get("gate", {})
    latency_threshold = float(gate.get("latency_regression_pct", 10.0))
    accuracy_threshold = float(gate.get("accuracy_regression_pct", 5.0))
    perf_metrics = [str(m) for m in gate.get("perf", {}).get("metrics", [])]
    loc_metrics = [str(m) for m in gate.get("localization", {}).get("metrics", [])]

    perf_baseline_map = _extract_perf_metric_map(_load_json(args.perf_baseline), args.node)
    perf_candidate_map = _extract_perf_metric_map(_load_json(args.perf_candidate), args.node)

    loc_baseline_map = (
        _extract_local_metric_map(_load_json(args.loc_baseline))
        if args.loc_baseline.exists()
        else {}
    )
    loc_candidate_map = (
        _extract_local_metric_map(_load_json(args.loc_candidate))
        if args.loc_candidate and args.loc_candidate.exists()
        else {}
    )

    results = []
    for metric in perf_metrics:
        results.append(
            _check_metric(
                metric_name=metric,
                baseline_value=_metric_value(perf_baseline_map, metric),
                candidate_value=_metric_value(perf_candidate_map, metric),
                threshold_pct=latency_threshold,
                category="latency",
            )
        )

    for metric in loc_metrics:
        threshold = latency_threshold if _is_latency_like_metric(metric) else accuracy_threshold
        category = "latency" if _is_latency_like_metric(metric) else "accuracy"
        results.append(
            _check_metric(
                metric_name=metric,
                baseline_value=_metric_value(loc_baseline_map, metric),
                candidate_value=_metric_value(loc_candidate_map, metric),
                threshold_pct=threshold,
                category=category,
            )
        )

    fail_count = sum(1 for item in results if item["status"] == "fail")
    pass_count = sum(1 for item in results if item["status"] == "pass")
    skipped_count = sum(1 for item in results if item["status"] == "skipped")
    overall_status = "fail" if fail_count > 0 else "pass"

    payload = {
        "version": 1,
        "generated_at": datetime.now().isoformat(),
        "overall_status": overall_status,
        "summary": {
            "fail_count": fail_count,
            "pass_count": pass_count,
            "skipped_count": skipped_count,
        },
        "results": results,
    }

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    _write_markdown(args.output_md, payload)

    print(f"[OK] gate result json: {args.output_json}")
    print(f"[OK] gate result markdown: {args.output_md}")
    print(f"[OK] overall_status: {overall_status}")

    return 2 if overall_status == "fail" else 0


if __name__ == "__main__":
    raise SystemExit(main())
