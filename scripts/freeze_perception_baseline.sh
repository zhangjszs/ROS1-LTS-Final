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
OUTPUT_BASELINE=""
FORCE=0

usage() {
  cat <<'EOF'
Usage:
  scripts/freeze_perception_baseline.sh --mode <track|accel|skidpad> --bag <bag_file> [options]

Required:
  --mode <mode>                 Mode name: track | accel | skidpad
  --bag <bag_file>              ROS bag path used to freeze baseline

Optional:
  --topic <topic>               Detection topic (default: /perception/lidar_cluster/detections)
  --gt <gt_csv>                 GT CSV path
  --gt-threshold <meters>       GT matching threshold (default: 1.0)
  --gt-max-range <meters>       GT max range (default: 50.0)
  --output <baseline_json>      Output baseline json path
  --force                       Overwrite existing output baseline
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
    --output)
      OUTPUT_BASELINE="${2:-}"
      shift 2
      ;;
    --force)
      FORCE=1
      shift
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
if [[ -n "${GT_FILE}" && ! -f "${GT_FILE}" ]]; then
  echo "ERROR: gt csv not found: ${GT_FILE}"
  exit 2
fi

mkdir -p "${BASELINE_DIR}"

if [[ -z "${OUTPUT_BASELINE}" ]]; then
  OUTPUT_BASELINE="${BASELINE_DIR}/${MODE}_baseline.json"
fi

if [[ -f "${OUTPUT_BASELINE}" && "${FORCE}" -ne 1 ]]; then
  echo "ERROR: output already exists: ${OUTPUT_BASELINE}"
  echo "       use --force to overwrite"
  exit 2
fi

source /opt/ros/noetic/setup.bash
if [[ -f "${ROOT_DIR}/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/devel/setup.bash"
fi

TMP_JSON="$(mktemp)"
cleanup() {
  rm -f "${TMP_JSON}"
}
trap cleanup EXIT

EVAL_CMD=(
  python3 "${ROOT_DIR}/perf_reports/scripts/evaluate_perception_metrics.py"
  "${BAG_FILE}"
  --topic "${TOPIC}"
  -o "${TMP_JSON}"
)
if [[ -n "${GT_FILE}" ]]; then
  EVAL_CMD+=(--gt "${GT_FILE}" --gt-threshold "${GT_THRESHOLD}" --gt-max-range "${GT_MAX_RANGE}")
fi

echo "[INFO] generating perception metrics for baseline freeze"
"${EVAL_CMD[@]}"

python3 - "${TMP_JSON}" "${OUTPUT_BASELINE}" "${MODE}" <<'PY'
import json
import sys

tmp_json, output_json, mode = sys.argv[1], sys.argv[2], sys.argv[3]

with open(tmp_json, "r", encoding="utf-8") as f:
    payload = json.load(f)

payload["mode"] = mode
payload["baseline_ready"] = True

with open(output_json, "w", encoding="utf-8") as f:
    json.dump(payload, f, indent=2, ensure_ascii=False)
    f.write("\n")
PY

echo "[OK] baseline frozen: ${OUTPUT_BASELINE}"
