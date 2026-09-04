#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BASELINE_DIR="${ROOT_DIR}/perf_reports/baselines/perception"
EVAL_SCRIPT="${ROOT_DIR}/perf_reports/scripts/evaluate_color_semantics_metrics.py"

MODE=""
BAG_FILE=""
TOPIC="/perception/fusion/detections"
OUTPUT_JSON=""

usage() {
  cat <<'EOF'
Usage:
  scripts/check_color_semantics_regression_mode.sh --mode <track|accel|skidpad> --bag <bag_file> [options]

Required:
  --mode <mode>                 Mode name: track | accel | skidpad
  --bag <bag_file>              Replay result bag path (contains fused detections)

Optional:
  --topic <topic>               Detection topic (default: /perception/fusion/detections)
  --output-json <path>          Metrics output json path
  --baseline-dir <path>         Threshold directory (default: perf_reports/baselines/perception)
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

if [[ ! -f "${BAG_FILE}" ]]; then
  echo "ERROR: bag file not found: ${BAG_FILE}"
  exit 2
fi

THRESHOLD_ENV="${BASELINE_DIR}/${MODE}.color_semantics.thresholds.env"
if [[ ! -f "${THRESHOLD_ENV}" ]]; then
  echo "ERROR: color semantics threshold file not found: ${THRESHOLD_ENV}"
  exit 2
fi

# shellcheck source=/dev/null
source "${THRESHOLD_ENV}"

if [[ -z "${OUTPUT_JSON}" ]]; then
  OUTPUT_JSON="${ROOT_DIR}/perf_reports/results/${MODE}_color_semantics_$(date +%Y%m%d_%H%M%S).json"
fi

echo "[INFO] mode=${MODE}"
echo "[INFO] bag=${BAG_FILE}"
echo "[INFO] topic=${TOPIC}"
echo "[INFO] thresholds=${THRESHOLD_ENV}"
echo "[INFO] output=${OUTPUT_JSON}"

CMD=(
  python3 "${EVAL_SCRIPT}"
  "${BAG_FILE}"
  --topic "${TOPIC}"
  --output-json "${OUTPUT_JSON}"
  --min-frames "${PERCEPTION_COLOR_MIN_FRAMES}"
  --min-blue-right "${PERCEPTION_COLOR_MIN_BLUE_RIGHT}"
  --min-left-boundary "${PERCEPTION_COLOR_MIN_LEFT_BOUNDARY}"
  --min-overall "${PERCEPTION_COLOR_MIN_OVERALL}"
  --max-none-rate "${PERCEPTION_COLOR_MAX_NONE_RATE}"
)

"${CMD[@]}"
