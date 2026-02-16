#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASELINE_DIR="${ROOT_DIR}/perf_reports/baselines/perception"

MODE=""
BAG_FILE=""
TOPIC="/perception/lidar_cluster/detections"
GT_FILE=""
GT_THRESHOLD="1.0"
GT_MAX_RANGE="50.0"
OUTPUT_JSON=""

usage() {
  cat <<'EOF'
Usage:
  scripts/check_perception_regression_mode.sh --mode <track|accel|skidpad> --bag <bag_file> [options]

Required:
  --mode <mode>                 Mode name: track | accel | skidpad
  --bag <bag_file>              ROS bag file path

Optional:
  --topic <topic>               Detection topic (default: /perception/lidar_cluster/detections)
  --gt <gt_csv>                 GT csv path
  --gt-threshold <meters>       GT matching threshold (default: 1.0)
  --gt-max-range <meters>       GT max range (default: 50.0)
  --output-json <path>          Current run output json path
  --baseline-dir <path>         Baseline directory (default: perf_reports/baselines/perception)
  -h, --help                    Show help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    --bag)
      BAG_FILE="${2:-}"
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
    --baseline-dir)
      BASELINE_DIR="${2:-}"
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

if [[ "${MODE}" != "track" && "${MODE}" != "accel" && "${MODE}" != "skidpad" ]]; then
  echo "ERROR: --mode must be one of: track, accel, skidpad"
  exit 2
fi

if [[ -z "${BAG_FILE}" ]]; then
  echo "ERROR: --bag is required"
  exit 2
fi

BASELINE_JSON="${BASELINE_DIR}/${MODE}_baseline.json"
THRESHOLD_ENV="${BASELINE_DIR}/${MODE}.thresholds.env"

if [[ -f "${THRESHOLD_ENV}" ]]; then
  # shellcheck source=/dev/null
  source "${THRESHOLD_ENV}"
fi

CMD=(
  "${ROOT_DIR}/scripts/check_perception_regression.sh"
  --bag "${BAG_FILE}"
  --baseline "${BASELINE_JSON}"
  --topic "${TOPIC}"
)
if [[ -n "${GT_FILE}" ]]; then
  CMD+=(--gt "${GT_FILE}" --gt-threshold "${GT_THRESHOLD}" --gt-max-range "${GT_MAX_RANGE}")
fi
if [[ -n "${OUTPUT_JSON}" ]]; then
  CMD+=(--output-json "${OUTPUT_JSON}")
fi

echo "[INFO] mode=${MODE}"
echo "[INFO] baseline=${BASELINE_JSON}"
if [[ -f "${THRESHOLD_ENV}" ]]; then
  echo "[INFO] thresholds=${THRESHOLD_ENV}"
fi
"${CMD[@]}"
