#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def _angle_diff_rad(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def _parse_bool(raw: str) -> Optional[bool]:
    if raw is None:
        return None
    text = str(raw).strip().lower()
    if text in {"1", "true", "t", "yes", "y"}:
        return True
    if text in {"0", "false", "f", "no", "n"}:
        return False
    return None


def _first_float(row: Dict[str, str], keys: List[str]) -> Optional[float]:
    for key in keys:
        if key in row and row[key] not in ("", None):
            try:
                return float(row[key])
            except ValueError:
                return None
    return None


def _resolve_timestamp(row: Dict[str, str]) -> Optional[float]:
    return _first_float(
        row,
        ["t_sec", "time_sec", "timestamp", "time", "stamp", "ts"],
    )


def _resolve_yaw_rad(row: Dict[str, str], prefix: str) -> Optional[float]:
    yaw_rad = _first_float(row, [f"{prefix}_yaw_rad", f"{prefix}_theta_rad"])
    if yaw_rad is not None:
        return yaw_rad

    yaw_deg = _first_float(row, [f"{prefix}_yaw_deg", f"{prefix}_theta_deg"])
    if yaw_deg is not None:
        return math.radians(yaw_deg)

    return None


def _read_rows(input_csv: Path) -> List[Dict[str, str]]:
    with input_csv.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return list(reader)


def _compute_rms(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return math.sqrt(sum(v * v for v in values) / len(values))


def _compute_recovery_from_valid(
    times: List[float], valid_series: List[Optional[bool]]
) -> Tuple[Optional[float], int, str]:
    if not valid_series or all(v is None for v in valid_series):
        return None, 0, "no_valid_column"

    recovery_times = []
    in_dropout = False
    dropout_start = None

    for idx, valid in enumerate(valid_series):
        if valid is None:
            continue
        t = times[idx]
        if not in_dropout and valid is False:
            in_dropout = True
            dropout_start = t
            continue
        if in_dropout and valid is True and dropout_start is not None:
            recovery_times.append(t - dropout_start)
            in_dropout = False
            dropout_start = None

    if not recovery_times:
        return 0.0, 0, "valid_column_no_dropout"
    mean_recovery = sum(recovery_times) / len(recovery_times)
    return mean_recovery, len(recovery_times), "valid_column"


def _compute_recovery_from_gaps(
    times: List[float], dropout_gap_s: float
) -> Tuple[Optional[float], int, str]:
    if len(times) < 2:
        return None, 0, "insufficient_samples"
    if dropout_gap_s <= 0.0:
        return None, 0, "gap_mode_disabled"

    gaps = []
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if dt > dropout_gap_s:
            gaps.append(dt)
    if not gaps:
        return 0.0, 0, "gap_mode_no_dropout"
    return sum(gaps) / len(gaps), len(gaps), "gap_mode"


def evaluate(input_csv: Path, dropout_gap_s: float) -> Dict:
    rows = _read_rows(input_csv)
    if not rows:
        raise RuntimeError(f"CSV has no rows: {input_csv}")

    pos_errors = []
    yaw_errors = []
    times = []
    valid_series: List[Optional[bool]] = []

    used_rows = 0
    for row in rows:
        t_sec = _resolve_timestamp(row)
        est_x = _first_float(row, ["est_x", "estimate_x", "x_est"])
        est_y = _first_float(row, ["est_y", "estimate_y", "y_est"])
        ref_x = _first_float(row, ["ref_x", "reference_x", "x_ref"])
        ref_y = _first_float(row, ["ref_y", "reference_y", "y_ref"])
        est_yaw = _resolve_yaw_rad(row, "est")
        ref_yaw = _resolve_yaw_rad(row, "ref")

        if None in (t_sec, est_x, est_y, ref_x, ref_y, est_yaw, ref_yaw):
            continue

        dx = est_x - ref_x
        dy = est_y - ref_y
        pos_errors.append(math.hypot(dx, dy))
        yaw_errors.append(_angle_diff_rad(est_yaw, ref_yaw))
        times.append(float(t_sec))
        valid_series.append(
            _parse_bool(
                row.get("est_valid", row.get("valid", row.get("estimate_valid", "")))
            )
        )
        used_rows += 1

    if used_rows == 0:
        raise RuntimeError(
            "No valid aligned samples found. "
            "Required columns include time, est/ref x/y, est/ref yaw."
        )

    position_rms = _compute_rms(pos_errors)
    heading_rms_rad = _compute_rms(yaw_errors)
    heading_rms_deg = math.degrees(heading_rms_rad) if heading_rms_rad is not None else None

    recovery_time, recovery_count, recovery_mode = _compute_recovery_from_valid(
        times, valid_series
    )

    if recovery_time is None:
        recovery_time, recovery_count, recovery_mode = _compute_recovery_from_gaps(
            times, dropout_gap_s
        )

    return {
        "version": 1,
        "generated_at": datetime.now().isoformat(),
        "source_csv": str(input_csv),
        "metrics": {
            "position_rms_m": position_rms,
            "heading_rms_deg": heading_rms_deg,
            "recovery_time_mean_s": recovery_time,
        },
        "details": {
            "row_count": len(rows),
            "aligned_sample_count": used_rows,
            "recovery_event_count": recovery_count,
            "recovery_mode": recovery_mode,
            "dropout_gap_s": dropout_gap_s,
        },
    }


def _write_markdown(output_md: Path, payload: Dict, label: str) -> None:
    output_md.parent.mkdir(parents=True, exist_ok=True)
    metrics = payload["metrics"]
    details = payload["details"]

    lines = []
    lines.append("# Localization Metrics Snapshot")
    lines.append("")
    lines.append(f"- label: `{label}`")
    lines.append(f"- generated_at: `{payload['generated_at']}`")
    lines.append(f"- source_csv: `{payload['source_csv']}`")
    lines.append(f"- aligned_sample_count: `{details['aligned_sample_count']}`")
    lines.append(
        f"- recovery_event_count: `{details['recovery_event_count']}` (`{details['recovery_mode']}`)"
    )
    lines.append("")
    lines.append("| metric | value |")
    lines.append("|---|---:|")
    lines.append(
        f"| position_rms_m | {metrics['position_rms_m']:.6f} |"
        if metrics["position_rms_m"] is not None
        else "| position_rms_m | N/A |"
    )
    lines.append(
        f"| heading_rms_deg | {metrics['heading_rms_deg']:.6f} |"
        if metrics["heading_rms_deg"] is not None
        else "| heading_rms_deg | N/A |"
    )
    lines.append(
        f"| recovery_time_mean_s | {metrics['recovery_time_mean_s']:.6f} |"
        if metrics["recovery_time_mean_s"] is not None
        else "| recovery_time_mean_s | N/A |"
    )
    output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate localization baseline metrics from aligned CSV. "
            "Expected columns include: "
            "t_sec, est_x, est_y, est_yaw_rad/deg, ref_x, ref_y, ref_yaw_rad/deg, "
            "optional est_valid/valid."
        )
    )
    parser.add_argument(
        "--input-csv", type=Path, required=True, help="Aligned localization evaluation CSV."
    )
    parser.add_argument(
        "--dropout-gap-s",
        type=float,
        default=0.2,
        help="Fallback dropout gap threshold (seconds) if no valid column exists.",
    )
    parser.add_argument(
        "--label",
        type=str,
        default="baseline",
        help="Snapshot label shown in markdown.",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=Path("docs/baseline/localization_baseline.json"),
        help="Output metrics json path.",
    )
    parser.add_argument(
        "--output-md",
        type=Path,
        default=Path("docs/baseline/localization_metrics.md"),
        help="Output markdown path.",
    )
    args = parser.parse_args()

    payload = evaluate(args.input_csv, args.dropout_gap_s)
    payload["label"] = args.label

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    _write_markdown(args.output_md, payload, args.label)

    print(f"[OK] localization metrics json: {args.output_json}")
    print(f"[OK] localization metrics markdown: {args.output_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
