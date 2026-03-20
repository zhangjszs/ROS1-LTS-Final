#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${OUT_DIR:-${ROOT_DIR}/perf_reports/results/localization_fg_mainline_$(date +%Y%m%d_%H%M%S)}"

MISSION="trackdrive"
BAG_PATH=""
PLAY_RATE="${PLAY_RATE:-2.0}"
BOOT_TIMEOUT_S="${BOOT_TIMEOUT_S:-40}"
LAUNCH_TIMEOUT_S="${LAUNCH_TIMEOUT_S:-180}"
COMPARE_DURATION_S="${COMPARE_DURATION_S:-60}"
MAX_SYNC_DIFF="${MAX_SYNC_DIFF:-0.10}"
MIN_SAMPLES="${MIN_SAMPLES:-80}"

LAUNCH_PID=""

usage() {
  cat <<'EOF'
Usage:
  scripts/validate_localization_fg_mainline_replay.sh [options]

Options:
  --mission <trackdrive|autocross|acceleration|skidpad>  Mission profile (default: trackdrive)
  --bag <path>                                            Replay bag path
  --rate <float>                                          rosbag play rate (default: 2.0)
  --out-dir <path>                                        Output directory
  --boot-timeout <sec>                                    ROS boot timeout (default: 40)
  --launch-timeout <sec>                                  Launch watchdog timeout (default: 180)
  --compare-duration <sec>                                Compare window (default: 60)
  --max-sync-diff <sec>                                   Compare max sync diff (default: 0.10)
  --min-samples <n>                                       Min compare samples for pass (default: 80)
  -h, --help                                              Show help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mission)
      MISSION="${2:-}"
      shift 2
      ;;
    --bag)
      BAG_PATH="${2:-}"
      shift 2
      ;;
    --rate)
      PLAY_RATE="${2:-}"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="${2:-}"
      shift 2
      ;;
    --boot-timeout)
      BOOT_TIMEOUT_S="${2:-}"
      shift 2
      ;;
    --launch-timeout)
      LAUNCH_TIMEOUT_S="${2:-}"
      shift 2
      ;;
    --compare-duration)
      COMPARE_DURATION_S="${2:-}"
      shift 2
      ;;
    --max-sync-diff)
      MAX_SYNC_DIFF="${2:-}"
      shift 2
      ;;
    --min-samples)
      MIN_SAMPLES="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

default_bag_for_mission() {
  case "$1" in
    trackdrive|autocross) echo "${HOME}/rosbag/track.bag" ;;
    acceleration) echo "${HOME}/rosbag/accel.bag" ;;
    skidpad) echo "${HOME}/rosbag/skidpad.bag" ;;
    *) echo "" ;;
  esac
}

mission_launch_path() {
  case "$1" in
    trackdrive|autocross|acceleration|skidpad)
      echo "${ROOT_DIR}/src/fsd_launch/launch/$1.launch"
      ;;
    *)
      echo ""
      ;;
  esac
}

source_workspace() {
  source /opt/ros/noetic/setup.bash
  if [[ -f "${ROOT_DIR}/devel/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${ROOT_DIR}/devel/setup.bash"
  elif [[ -f "${ROOT_DIR}/devel/setup.sh" ]]; then
    # shellcheck source=/dev/null
    source "${ROOT_DIR}/devel/setup.sh"
  fi
}

kill_launch() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
    LAUNCH_PID=""
  fi
}

cleanup() {
  kill_launch
}
trap cleanup EXIT

if [[ -z "${BAG_PATH}" ]]; then
  BAG_PATH="$(default_bag_for_mission "${MISSION}")"
fi
if [[ -z "${BAG_PATH}" ]]; then
  echo "[ERROR] cannot infer bag path for mission=${MISSION}, please set --bag"
  exit 2
fi
if [[ ! -f "${BAG_PATH}" ]]; then
  echo "[ERROR] bag not found: ${BAG_PATH}"
  exit 2
fi

MISSION_LAUNCH="$(mission_launch_path "${MISSION}")"
if [[ -z "${MISSION_LAUNCH}" || ! -f "${MISSION_LAUNCH}" ]]; then
  echo "[ERROR] mission launch not found for mission=${MISSION}"
  exit 2
fi

source_workspace
mkdir -p "${OUT_DIR}"

LAUNCH_LOG="${OUT_DIR}/launch.log"
DIAG_LOG="${OUT_DIR}/diagnostics.log"
COMPARE_CSV="${OUT_DIR}/backend_comparison.csv"
COMPARE_PNG="${OUT_DIR}/backend_comparison.png"
RESULT_TSV="${OUT_DIR}/results.tsv"
SUMMARY_MD="${OUT_DIR}/summary.md"
RESULT_JSON="${OUT_DIR}/result.json"

rm -f "${LAUNCH_LOG}" "${DIAG_LOG}" "${COMPARE_CSV}" "${COMPARE_PNG}"

echo "[fg-mainline] mission=${MISSION} bag=${BAG_PATH}" | tee -a "${LAUNCH_LOG}"
timeout "${LAUNCH_TIMEOUT_S}s" roslaunch "${MISSION_LAUNCH}" \
  simulation:=true \
  bag:="${BAG_PATH}" \
  rate:="${PLAY_RATE}" \
  loop:=false \
  launch_rviz:=false \
  launch_viz:=false \
  backend:=factor_graph \
  publish_dual_backends:=true \
  fg_shadow_mode:=false \
  fg_mainline_enable_mapper_fallback:=true \
  > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 "${BOOT_TIMEOUT_S}"); do
  if rostopic list >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
if ! rostopic list >/dev/null 2>&1; then
  printf "mission\tstatus\tsamples\tbackend_factor_graph_seen\tactive_source_fg_seen\tfg_shadow_false_seen\tnotes\n" \
    > "${RESULT_TSV}"
  printf "%s\tFAIL\t0\tno\tno\tno\tros master not ready\n" "${MISSION}" >> "${RESULT_TSV}"
  cat > "${SUMMARY_MD}" <<EOF
# Localization FG Mainline Validation

| mission | status | samples | backend_factor_graph_seen | active_source_fg_seen | fg_shadow_false_seen | notes |
|---|---|---:|---|---|---|---|
| ${MISSION} | FAIL | 0 | no | no | no | ros master not ready |
EOF
  cat > "${RESULT_JSON}" <<EOF
{"mission":"${MISSION}","status":"FAIL","samples":0,"notes":"ros master not ready"}
EOF
  kill_launch
  exit 1
fi

# Ensure key topics can be observed at least once.
for topic in /localization/car_state /localization/mapper/car_state /localization/fg/car_state; do
  if ! timeout 25s rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "[fg-mainline][WARN] topic missing sample: ${topic}" | tee -a "${LAUNCH_LOG}"
  fi
done

timeout "$((COMPARE_DURATION_S + 20))s" rostopic echo /localization/diagnostics > "${DIAG_LOG}" 2>/dev/null &
DIAG_PID=$!

compare_status="PASS"
if ! python3 "${ROOT_DIR}/perf_reports/scripts/compare_backends.py" \
  --duration "${COMPARE_DURATION_S}" \
  --output "${COMPARE_CSV}" \
  --plot-output "${COMPARE_PNG}" \
  --max-sync-diff "${MAX_SYNC_DIFF}" \
  --mapper-topic "/localization/mapper/car_state" \
  --fg-topic "/localization/fg/car_state"; then
  compare_status="FAIL"
fi

kill "${DIAG_PID}" 2>/dev/null || true
wait "${DIAG_PID}" 2>/dev/null || true
kill_launch

python3 - "${MISSION}" "${COMPARE_CSV}" "${DIAG_LOG}" "${RESULT_TSV}" "${SUMMARY_MD}" "${RESULT_JSON}" "${MIN_SAMPLES}" "${compare_status}" <<'PY'
import csv
import json
import math
import pathlib
import re
import sys

mission = sys.argv[1]
csv_path = pathlib.Path(sys.argv[2])
diag_path = pathlib.Path(sys.argv[3])
result_tsv = pathlib.Path(sys.argv[4])
summary_md = pathlib.Path(sys.argv[5])
result_json = pathlib.Path(sys.argv[6])
min_samples = int(sys.argv[7])
compare_status = sys.argv[8]

rows = []
if csv_path.exists():
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

def p95(values):
    if not values:
        return float("nan")
    vals = sorted(values)
    idx = int(math.ceil(0.95 * len(vals))) - 1
    idx = min(max(idx, 0), len(vals) - 1)
    return vals[idx]

pos = []
heading = []
vel = []
tdiff = []
for r in rows:
    try:
        pos.append(float(r["pos_error_m"]))
        heading.append(float(r["heading_error_deg"]))
        vel.append(float(r["velocity_error_mps"]))
        tdiff.append(float(r["time_diff_s"]))
    except Exception:
        continue

diag_text = diag_path.read_text(encoding="utf-8", errors="ignore") if diag_path.exists() else ""
backend_factor_graph_seen = bool(
    re.search(r'key:\s*"backend"\s*\n\s*value:\s*"factor_graph"', diag_text)
)
active_source_fg_seen = bool(
    re.search(r'key:\s*"active_backend_source"\s*\n\s*value:\s*"factor_graph"', diag_text)
)
fg_shadow_false_seen = bool(
    re.search(r'key:\s*"fg_shadow_mode"\s*\n\s*value:\s*"false"', diag_text)
)

status = "PASS"
notes = []
if compare_status != "PASS":
    status = "FAIL"
    notes.append("compare_backends failed")
if len(rows) < min_samples:
    status = "FAIL"
    notes.append(f"samples<{min_samples}")
if not backend_factor_graph_seen:
    status = "FAIL"
    notes.append("backend!=factor_graph in diagnostics")
if not active_source_fg_seen:
    status = "FAIL"
    notes.append("active_backend_source!=factor_graph in diagnostics")
if not fg_shadow_false_seen:
    status = "FAIL"
    notes.append("fg_shadow_mode!=false in diagnostics")

if not notes:
    notes = ["ok"]

payload = {
    "mission": mission,
    "status": status,
    "samples": len(rows),
    "metrics": {
        "pos_error_p95_m": p95(pos),
        "heading_error_p95_deg": p95(heading),
        "velocity_error_p95_mps": p95(vel),
        "time_diff_p95_s": p95(tdiff),
    },
    "checks": {
        "backend_factor_graph_seen": backend_factor_graph_seen,
        "active_source_fg_seen": active_source_fg_seen,
        "fg_shadow_false_seen": fg_shadow_false_seen,
    },
    "notes": "; ".join(notes),
}

result_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

result_tsv.write_text(
    "mission\tstatus\tsamples\tbackend_factor_graph_seen\tactive_source_fg_seen\tfg_shadow_false_seen\tpos_p95_m\theading_p95_deg\tvel_p95_mps\ttime_diff_p95_s\tnotes\n"
    f"{mission}\t{status}\t{len(rows)}\t"
    f"{'yes' if backend_factor_graph_seen else 'no'}\t"
    f"{'yes' if active_source_fg_seen else 'no'}\t"
    f"{'yes' if fg_shadow_false_seen else 'no'}\t"
    f"{payload['metrics']['pos_error_p95_m']:.6f}\t"
    f"{payload['metrics']['heading_error_p95_deg']:.6f}\t"
    f"{payload['metrics']['velocity_error_p95_mps']:.6f}\t"
    f"{payload['metrics']['time_diff_p95_s']:.6f}\t"
    f"{payload['notes']}\n",
    encoding="utf-8",
)

summary_md.write_text(
    "# Localization FG Mainline Validation\n\n"
    "| mission | status | samples | backend_factor_graph_seen | active_source_fg_seen | "
    "fg_shadow_false_seen | pos_p95_m | heading_p95_deg | vel_p95_mps | time_diff_p95_s | notes |\n"
    "|---|---|---:|---|---|---|---:|---:|---:|---:|---|\n"
    f"| {mission} | {status} | {len(rows)} | "
    f"{'yes' if backend_factor_graph_seen else 'no'} | "
    f"{'yes' if active_source_fg_seen else 'no'} | "
    f"{'yes' if fg_shadow_false_seen else 'no'} | "
    f"{payload['metrics']['pos_error_p95_m']:.4f} | "
    f"{payload['metrics']['heading_error_p95_deg']:.4f} | "
    f"{payload['metrics']['velocity_error_p95_mps']:.4f} | "
    f"{payload['metrics']['time_diff_p95_s']:.4f} | {payload['notes']} |\n",
    encoding="utf-8",
)

print(status)
PY

echo "[fg-mainline] results: ${RESULT_TSV}"
echo "[fg-mainline] summary: ${SUMMARY_MD}"
echo "[fg-mainline] json: ${RESULT_JSON}"

if awk -F '\t' 'NR>1 && $2 != "PASS" {exit 1}' "${RESULT_TSV}"; then
  echo "[fg-mainline] ACCEPTANCE PASS"
else
  echo "[fg-mainline] ACCEPTANCE FAIL"
  exit 1
fi
