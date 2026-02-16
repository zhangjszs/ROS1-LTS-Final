#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BAG_FILE=""
BASELINE_JSON=""
TOPIC="/perception/lidar_cluster/detections"
GT_FILE=""
GT_THRESHOLD="1.0"
GT_MAX_RANGE="50.0"
OUTPUT_JSON=""

usage() {
  cat <<'EOF'
Usage:
  scripts/check_perception_regression.sh --bag <bag_file> --baseline <baseline_json> [options]

Required:
  --bag <bag_file>            ROS bag file path
  --baseline <baseline_json>  Baseline metrics JSON from evaluate_perception_metrics.py

Optional:
  --topic <topic>             Detection topic (default: /perception/lidar_cluster/detections)
  --gt <gt_csv>               GT csv file (enables GT metrics comparison)
  --gt-threshold <meters>     GT matching threshold (default: 1.0)
  --gt-max-range <meters>     GT max range (default: 50.0)
  --output-json <path>        Current run output json path
  -h, --help                  Show this help

Threshold env vars:
  PERCEPTION_MAX_REL_INCREASE   Allowed increase for lower-better metrics (default: 0.20)
  PERCEPTION_MAX_REL_DECREASE   Allowed decrease for higher-better metrics (default: 0.10)
  PERCEPTION_MAX_ABS_WHEN_ZERO  Allowed absolute value if baseline is 0 (default: 0.02)
  PERCEPTION_MIN_FRAME_RATIO    Minimum n_frames ratio vs baseline (default: 0.95)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG_FILE="${2:-}"
      shift 2
      ;;
    --baseline)
      BASELINE_JSON="${2:-}"
      shift 2
      ;;
    --topic)
      TOPIC="${2:-}"
      shift 2
      ;;
    --gt)
      GT_FILE="${2:-}"
      shift 2
      ;;
    --gt-threshold)
      GT_THRESHOLD="${2:-}"
      shift 2
      ;;
    --gt-max-range)
      GT_MAX_RANGE="${2:-}"
      shift 2
      ;;
    --output-json)
      OUTPUT_JSON="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

if [[ -z "${BAG_FILE}" || -z "${BASELINE_JSON}" ]]; then
  echo "ERROR: --bag and --baseline are required"
  usage
  exit 2
fi

if [[ ! -f "${BAG_FILE}" ]]; then
  echo "ERROR: bag file not found: ${BAG_FILE}"
  exit 2
fi
if [[ ! -f "${BASELINE_JSON}" ]]; then
  echo "ERROR: baseline json not found: ${BASELINE_JSON}"
  exit 2
fi
if [[ -n "${GT_FILE}" && ! -f "${GT_FILE}" ]]; then
  echo "ERROR: gt csv not found: ${GT_FILE}"
  exit 2
fi

source /opt/ros/noetic/setup.bash
if [[ -f "${ROOT_DIR}/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/devel/setup.bash"
fi

TMP_DIR=""
if [[ -z "${OUTPUT_JSON}" ]]; then
  TMP_DIR="$(mktemp -d)"
  OUTPUT_JSON="${TMP_DIR}/perception_current_metrics.json"
fi

cleanup() {
  if [[ -n "${TMP_DIR}" ]]; then
    rm -rf "${TMP_DIR}"
  fi
}
trap cleanup EXIT

EVAL_CMD=(
  python3 "${ROOT_DIR}/perf_reports/scripts/evaluate_perception_metrics.py"
  "${BAG_FILE}"
  --topic "${TOPIC}"
  -o "${OUTPUT_JSON}"
)
if [[ -n "${GT_FILE}" ]]; then
  EVAL_CMD+=(--gt "${GT_FILE}" --gt-threshold "${GT_THRESHOLD}" --gt-max-range "${GT_MAX_RANGE}")
fi

echo "[INFO] running perception evaluation"
"${EVAL_CMD[@]}"

echo "[INFO] comparing against baseline"
python3 - "${BASELINE_JSON}" "${OUTPUT_JSON}" <<'PY'
import json
import os
import sys

baseline_path, current_path = sys.argv[1], sys.argv[2]

with open(baseline_path, "r", encoding="utf-8") as f:
    baseline = json.load(f)
with open(current_path, "r", encoding="utf-8") as f:
    current = json.load(f)

baseline_metrics = baseline.get("metrics", {})
current_metrics = current.get("metrics", {})

if baseline.get("baseline_ready", True) is False:
    print("[RESULT] perception regression check FAILED")
    print("[FAIL] baseline is not ready (baseline_ready=false); freeze baseline first")
    print(f"[INFO] baseline: {baseline_path}")
    sys.exit(1)

required_metrics = [
    "n_frames",
    "mean_detections",
    "std_detections",
    "spike_rate",
    "zero_frame_rate",
    "mean_confidence",
    "std_confidence",
    "mean_distance_m",
    "symmetry_ratio",
    "mean_frame_interval_s",
]

def _missing_required(metric_map):
    missing = []
    for name in required_metrics:
        value = metric_map.get(name)
        if not isinstance(value, (int, float)):
            missing.append(name)
    return missing

missing_baseline = _missing_required(baseline_metrics)
if missing_baseline:
    print("[RESULT] perception regression check FAILED")
    print("[FAIL] baseline is missing required numeric metrics")
    print(f"[INFO] missing baseline metrics: {', '.join(missing_baseline)}")
    print(f"[INFO] baseline: {baseline_path}")
    sys.exit(1)

missing_current = _missing_required(current_metrics)
if missing_current:
    print("[RESULT] perception regression check FAILED")
    print("[FAIL] current run is missing required numeric metrics")
    print(f"[INFO] missing current metrics: {', '.join(missing_current)}")
    print(f"[INFO] current: {current_path}")
    sys.exit(1)

lower_better_metrics = [
    "std_detections",
    "spike_rate",
    "zero_frame_rate",
    "std_confidence",
    "mean_frame_interval_s",
]
higher_better_metrics = [
    "mean_confidence",
]

for metric in ("precision", "recall", "f1"):
    if metric in baseline_metrics and metric in current_metrics:
        higher_better_metrics.append(metric)
if "rmse_m" in baseline_metrics and "rmse_m" in current_metrics:
    lower_better_metrics.append("rmse_m")

max_rel_increase = float(os.getenv("PERCEPTION_MAX_REL_INCREASE", "0.20"))
max_rel_decrease = float(os.getenv("PERCEPTION_MAX_REL_DECREASE", "0.10"))
max_abs_when_zero = float(os.getenv("PERCEPTION_MAX_ABS_WHEN_ZERO", "0.02"))
min_frame_ratio = float(os.getenv("PERCEPTION_MIN_FRAME_RATIO", "0.95"))

failures = []
infos = []

def _fmt(v):
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)

baseline_frames = baseline_metrics.get("n_frames")
current_frames = current_metrics.get("n_frames")
if isinstance(baseline_frames, (int, float)) and isinstance(current_frames, (int, float)) and baseline_frames > 0:
    min_allowed = baseline_frames * min_frame_ratio
    infos.append(
        f"n_frames: baseline={_fmt(baseline_frames)} current={_fmt(current_frames)} min_allowed={_fmt(min_allowed)}"
    )
    if current_frames < min_allowed:
        failures.append(
            f"n_frames dropped below threshold: baseline={_fmt(baseline_frames)} current={_fmt(current_frames)} min_allowed={_fmt(min_allowed)}"
        )

for metric in lower_better_metrics:
    if metric not in baseline_metrics or metric not in current_metrics:
        continue
    baseline_value = float(baseline_metrics[metric])
    current_value = float(current_metrics[metric])
    if baseline_value == 0.0:
        allowed = max_abs_when_zero
    else:
        allowed = baseline_value * (1.0 + max_rel_increase)
    infos.append(
        f"{metric}: baseline={_fmt(baseline_value)} current={_fmt(current_value)} max_allowed={_fmt(allowed)}"
    )
    if current_value > allowed:
        failures.append(
            f"{metric} regressed (higher is worse): baseline={_fmt(baseline_value)} current={_fmt(current_value)} max_allowed={_fmt(allowed)}"
        )

for metric in higher_better_metrics:
    if metric not in baseline_metrics or metric not in current_metrics:
        continue
    baseline_value = float(baseline_metrics[metric])
    current_value = float(current_metrics[metric])
    if baseline_value <= 0.0:
        infos.append(
            f"{metric}: baseline={_fmt(baseline_value)} current={_fmt(current_value)} (skip threshold, non-positive baseline)"
        )
        continue
    min_allowed = baseline_value * (1.0 - max_rel_decrease)
    infos.append(
        f"{metric}: baseline={_fmt(baseline_value)} current={_fmt(current_value)} min_allowed={_fmt(min_allowed)}"
    )
    if current_value < min_allowed:
        failures.append(
            f"{metric} regressed (lower is worse): baseline={_fmt(baseline_value)} current={_fmt(current_value)} min_allowed={_fmt(min_allowed)}"
        )

if failures:
    print("[RESULT] perception regression check FAILED")
    for item in failures:
        print(f"[FAIL] {item}")
    print(f"[INFO] baseline: {baseline_path}")
    print(f"[INFO] current:  {current_path}")
    sys.exit(1)

print("[RESULT] perception regression check PASSED")
for item in infos:
    print(f"[INFO] {item}")
print(f"[INFO] baseline: {baseline_path}")
print(f"[INFO] current:  {current_path}")
PY
