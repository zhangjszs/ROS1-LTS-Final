#!/usr/bin/env python3
"""Check localization FG mainline result against hard gate thresholds."""

import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


def _load_yaml(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _load_json(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _metric_value(data: Dict, key: str) -> Optional[float]:
    raw = data.get(key)
    if raw is None:
        return None
    if isinstance(raw, (int, float)):
        return float(raw)
    return None


def _fmt(v: Any) -> str:
    if v is None:
        return "N/A"
    if isinstance(v, str):
        return v
    if isinstance(v, (int, float)):
        if math.isinf(v):
            return "INF"
        return f"{v:.4f}"
    return str(v)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check localization FG acceptance result against hard thresholds."
    )
    parser.add_argument(
        "--result-json",
        type=Path,
        required=True,
        help="Path to result.json produced by validate_localization_fg_mainline_replay.sh",
    )
    parser.add_argument(
        "--thresholds",
        type=Path,
        default=Path("perf_reports/baselines/localization/fg_gate_thresholds.yaml"),
        help="Path to gate thresholds YAML.",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=Path("perf_reports/results/fg_gate_result.json"),
        help="Output gate result JSON.",
    )
    parser.add_argument(
        "--output-md",
        type=Path,
        default=Path("perf_reports/results/fg_gate_result.md"),
        help="Output gate result Markdown.",
    )
    args = parser.parse_args()

    if not args.result_json.exists():
        print(f"[ERROR] result json not found: {args.result_json}", file=sys.stderr)
        return 2

    if not args.thresholds.exists():
        print(f"[ERROR] thresholds file not found: {args.thresholds}", file=sys.stderr)
        return 2

    result = _load_json(args.result_json)
    thresholds_cfg = _load_yaml(args.thresholds)

    mission = result.get("mission", "unknown")
    status = result.get("status", "UNKNOWN")
    samples = result.get("samples", 0)
    metrics = result.get("metrics", {})

    mission_thresholds = thresholds_cfg.get("missions", {}).get(mission, {})
    if not mission_thresholds:
        print(f"[ERROR] no thresholds defined for mission={mission}", file=sys.stderr)
        return 2

    checks: List[Dict[str, Any]] = []
    overall_fail = False

    # 1. Overall status must be PASS
    if status != "PASS":
        overall_fail = True
        checks.append(
            {
                "check": "overall_status",
                "value": status,
                "threshold": "PASS",
                "status": "fail",
                "reason": f"result status is {status}, expected PASS",
            }
        )
    else:
        checks.append(
            {
                "check": "overall_status",
                "value": status,
                "threshold": "PASS",
                "status": "pass",
                "reason": "",
            }
        )

    # 2. Min samples
    min_samples = mission_thresholds.get("min_samples", 0)
    if samples < min_samples:
        overall_fail = True
        checks.append(
            {
                "check": "min_samples",
                "value": samples,
                "threshold": f">= {min_samples}",
                "status": "fail",
                "reason": f"samples {samples} < min {min_samples}",
            }
        )
    else:
        checks.append(
            {
                "check": "min_samples",
                "value": samples,
                "threshold": f">= {min_samples}",
                "status": "pass",
                "reason": "",
            }
        )

    # 3. Metric thresholds (max_* keys map to metrics names)
    metric_map = {
        "max_position_p95_m": "pos_error_p95_m",
        "max_heading_p95_deg": "heading_error_p95_deg",
        "max_velocity_p95_mps": "velocity_error_p95_mps",
        "max_time_diff_p95_s": "time_diff_p95_s",
    }

    for thresh_key, metric_key in metric_map.items():
        threshold = mission_thresholds.get(thresh_key)
        if threshold is None:
            continue
        value = _metric_value(metrics, metric_key)
        if value is None:
            checks.append(
                {
                    "check": metric_key,
                    "value": None,
                    "threshold": f"<= {threshold}",
                    "status": "skipped",
                    "reason": "missing in result metrics",
                }
            )
            continue

        if value > threshold:
            overall_fail = True
            checks.append(
                {
                    "check": metric_key,
                    "value": value,
                    "threshold": f"<= {threshold}",
                    "status": "fail",
                    "reason": f"{_fmt(value)} > {_fmt(threshold)}",
                }
            )
        else:
            checks.append(
                {
                    "check": metric_key,
                    "value": value,
                    "threshold": f"<= {threshold}",
                    "status": "pass",
                    "reason": "",
                }
            )

    # Build payload
    payload = {
        "version": 1,
        "generated_at": datetime.now().isoformat(),
        "mission": mission,
        "overall_status": "fail" if overall_fail else "pass",
        "checks": checks,
    }

    # Write JSON
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    # Write Markdown
    args.output_md.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Localization FG Gate Result",
        "",
        f"- **mission**: `{mission}`",
        f"- **overall_status**: `{payload['overall_status']}`",
        f"- **generated_at**: `{payload['generated_at']}`",
        "",
        "| check | value | threshold | status | reason |",
        "|---|---|---|---|---|",
    ]
    for c in checks:
        val = _fmt(c["value"]) if c["value"] is not None else "N/A"
        lines.append(
            f"| {c['check']} | {val} | {c['threshold']} | {c['status']} | {c.get('reason', '')} |"
        )
    args.output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"[OK] gate result json: {args.output_json}")
    print(f"[OK] gate result markdown: {args.output_md}")
    print(f"[OK] overall_status: {payload['overall_status']}")

    return 1 if overall_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
