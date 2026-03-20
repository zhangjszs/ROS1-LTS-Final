#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MISSION_STACK_LAUNCH="${ROOT_DIR}/src/fsd_launch/launch/subsystems/mission_stack.launch"
OUT_DIR="${OUT_DIR:-${ROOT_DIR}/perf_reports/data/pathlimits_v2_cutover_$(date +%Y%m%d_%H%M%S)}"
MISSION="trackdrive"
BAG_PATH=""
PLAY_RATE="${PLAY_RATE:-2.0}"
BOOT_TIMEOUT_S="${BOOT_TIMEOUT_S:-40}"
SCENARIO_TIMEOUT_S="${SCENARIO_TIMEOUT_S:-120}"
PATHLIMITS_WAIT_S="${PATHLIMITS_WAIT_S:-60}"

LAUNCH_PID=""

usage() {
  cat <<'EOF'
Usage:
  scripts/validate_pathlimits_v2_cutover_replay.sh [options]

Options:
  --mission <trackdrive|autocross|acceleration|skidpad>  Mission profile (default: trackdrive)
  --bag <path>                                            Replay bag path
  --rate <float>                                          rosbag play rate (default: 2.0)
  --out-dir <path>                                        Output directory
  --boot-timeout <sec>                                    ROS boot timeout (default: 40)
  --scenario-timeout <sec>                                Per-scenario launch timeout (default: 120)
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
    --scenario-timeout)
      SCENARIO_TIMEOUT_S="${2:-}"
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
    *)
      echo ""
      ;;
  esac
}

mission_profile() {
  case "$MISSION" in
    trackdrive|autocross)
      PERCEPTION_MODE="track"
      LOCALIZATION_MODE="track"
      PLANNING_MISSION="high_speed"
      CONTROL_MODE_PROFILE="track"
      CONTROL_MODE="4"
      DECISION_BUDGET="0.12"
      ;;
    acceleration)
      PERCEPTION_MODE="accel"
      LOCALIZATION_MODE="accel"
      PLANNING_MISSION="acceleration"
      CONTROL_MODE_PROFILE="accel"
      CONTROL_MODE="2"
      DECISION_BUDGET="0.10"
      ;;
    skidpad)
      PERCEPTION_MODE="skidpad"
      LOCALIZATION_MODE="skidpad"
      PLANNING_MISSION="skidpad"
      CONTROL_MODE_PROFILE="skidpad"
      CONTROL_MODE="3"
      DECISION_BUDGET="0.15"
      ;;
    *)
      echo "[ERROR] unsupported mission: ${MISSION}"
      exit 2
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

extract_last_diag_value() {
  local diag_path="$1"
  local key="$2"
  python3 - "${diag_path}" "${key}" <<'PY'
import pathlib
import re
import sys

diag_path = pathlib.Path(sys.argv[1])
key = sys.argv[2]
try:
    text = diag_path.read_text(encoding="utf-8", errors="ignore")
except FileNotFoundError:
    raise SystemExit(1)
pattern = re.compile(r"key:\s*{}\s*\n\s*value:\s*['\"]?([^'\n\"]+)".format(re.escape(key)))
matches = pattern.findall(text)
if not matches:
    raise SystemExit(1)
print(matches[-1])
PY
}

extract_source_from_launch_log() {
  local log_path="$1"
  python3 - "${log_path}" <<'PY'
import re
import sys

log_path = sys.argv[1]
text = open(log_path, "r", encoding="utf-8", errors="ignore").read()
best_source = None
best_ts = -1.0
best_idx = -1
for idx, line in enumerate(text.splitlines()):
    candidate = None
    selected = re.search(r"PathLimits source selected:\s*([A-Za-z0-9_-]+)", line)
    if selected:
        candidate = selected.group(1)
    switched = re.search(r"PathLimits source switch:.*->\s*([A-Za-z0-9_-]+)", line)
    if switched:
        candidate = switched.group(1)
    recovered = re.search(r"PathLimitsV2 recovered; switching back to V2 primary", line)
    if recovered:
        candidate = "v2"
    fallback = re.search(r"PathLimitsV2 stale or missing; fallback to V1", line)
    if fallback:
        candidate = "v1-fallback"
    if candidate is None:
        continue

    ts_match = re.search(r"\[([0-9]+\.[0-9]+)", line)
    ts = float(ts_match.group(1)) if ts_match else -1.0
    if ts > best_ts or (ts == best_ts and idx > best_idx):
        best_ts = ts
        best_idx = idx
        best_source = candidate

if best_source is None:
    raise SystemExit(1)
print(best_source)
PY
}

append_result() {
  local scenario="$1"
  local status="$2"
  local active_source="$3"
  local fallback_count="$4"
  local recover_count="$5"
  local v1_count="$6"
  local v2_count="$7"
  local v2_seen="$8"
  local notes="$9"
  printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" \
    "${scenario}" "${status}" "${active_source}" "${fallback_count}" "${recover_count}" \
    "${v1_count}" "${v2_count}" "${v2_seen}" "${notes}" >> "${RESULT_TSV}"
}

run_scenario() {
  local scenario="$1"
  local expect_source="$2"
  local expect_v2="$3"
  shift 3
  local extra_args=("$@")

  local scenario_dir="${OUT_DIR}/${scenario}"
  local launch_log="${scenario_dir}/launch.log"
  local diag_log="${scenario_dir}/diagnostics.log"
  mkdir -p "${scenario_dir}"
  rm -f "${launch_log}" "${diag_log}"

  local master_port="$((22000 + RANDOM % 20000))"
  local diag_pid=""
  export ROS_LOG_DIR="${scenario_dir}/roslog"
  export ROS_MASTER_URI="http://127.0.0.1:${master_port}"
  export ROS_HOSTNAME="127.0.0.1"
  mkdir -p "${ROS_LOG_DIR}"

  echo "[pathlimits-v2] scenario=${scenario} mission=${MISSION} bag=${BAG_PATH}" | tee -a "${launch_log}"
  timeout "${SCENARIO_TIMEOUT_S}s" roslaunch "${MISSION_STACK_LAUNCH}" \
    simulation:=true \
    bag:="${BAG_PATH}" \
    rate:="${PLAY_RATE}" \
    loop:=false \
    launch_rviz:=false \
    launch_viz:=false \
    perception_mode:="${PERCEPTION_MODE}" \
    localization_mode:="${LOCALIZATION_MODE}" \
    planning_mission:="${PLANNING_MISSION}" \
    control_mode_profile:="${CONTROL_MODE_PROFILE}" \
    control_mode:="${CONTROL_MODE}" \
    decision_fusion_budget_sec:="${DECISION_BUDGET}" \
    "${extra_args[@]}" \
    > "${launch_log}" 2>&1 &
  LAUNCH_PID=$!

  for _ in $(seq 1 "${BOOT_TIMEOUT_S}"); do
    if rostopic list >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
  if ! rostopic list >/dev/null 2>&1; then
    append_result "${scenario}" "FAIL" "__BOOT_FAILED__" "0" "0" "0" "0" "unknown" "ros master not ready"
    kill_launch
    return
  fi

  timeout "${SCENARIO_TIMEOUT_S}s" rostopic echo /control/diagnostics > "${diag_log}" 2>/dev/null &
  diag_pid=$!

  if ! timeout "${PATHLIMITS_WAIT_S}s" rostopic echo -n 1 /planning/pathlimits >/dev/null 2>&1; then
    if [[ -n "${diag_pid}" ]]; then
      kill "${diag_pid}" 2>/dev/null || true
      wait "${diag_pid}" 2>/dev/null || true
      diag_pid=""
    fi
    append_result "${scenario}" "FAIL" "__NO_V1_PATHLIMITS__" "0" "0" "0" "0" "unknown" "no /planning/pathlimits"
    kill_launch
    return
  fi

  local v2_seen="no"
  if [[ "${expect_v2}" == "yes" ]]; then
    if timeout "${PATHLIMITS_WAIT_S}s" rostopic echo -n 1 /planning/pathlimits_v2 >/dev/null 2>&1; then
      v2_seen="yes"
      sleep 2
    fi
  else
    if timeout 8s rostopic echo -n 1 /planning/pathlimits_v2 >/dev/null 2>&1; then
      v2_seen="yes"
    fi
  fi

  sleep 2
  if [[ -n "${diag_pid}" ]]; then
    kill "${diag_pid}" 2>/dev/null || true
    wait "${diag_pid}" 2>/dev/null || true
    diag_pid=""
  fi
  kill_launch

  local active_source="__MISSING__"
  local fallback_count="0"
  local recover_count="0"
  local v1_count="0"
  local v2_count="0"

  if [[ -s "${diag_log}" ]]; then
    active_source="$(extract_last_diag_value "${diag_log}" "active_pathlimits_source" || echo "__MISSING__")"
    fallback_count="$(extract_last_diag_value "${diag_log}" "pathlimits_v2_fallback_count" || echo "0")"
    recover_count="$(extract_last_diag_value "${diag_log}" "pathlimits_v2_recover_count" || echo "0")"
    v1_count="$(extract_last_diag_value "${diag_log}" "pathlimits_v1_msg_count" || echo "0")"
    v2_count="$(extract_last_diag_value "${diag_log}" "pathlimits_v2_msg_count" || echo "0")"
  fi
  local active_source_from_log="__MISSING__"
  active_source_from_log="$(extract_source_from_launch_log "${launch_log}" || echo "__MISSING__")"
  if [[ "${active_source_from_log}" != "__MISSING__" ]]; then
    active_source="${active_source_from_log}"
  elif [[ "${active_source}" == "__MISSING__" ]]; then
    active_source="__MISSING__"
  fi

  local v2_seen_evidence="none"
  if [[ "${expect_v2}" == "yes" && "${v2_seen}" != "yes" ]]; then
    if [[ "${v2_count}" =~ ^[0-9]+$ ]] && (( v2_count > 0 )); then
      v2_seen="yes"
      v2_seen_evidence="diagnostics_count"
    elif rg -q "PathLimitsV2 recovered; switching back to V2 primary|PathLimits source selected: v2|PathLimits source switch:.*->\\s*v2" "${launch_log}"; then
      v2_seen="yes"
      v2_seen_evidence="launch_log"
    fi
  fi

  local status="PASS"
  local notes="ok"
  if [[ "${v2_seen_evidence}" != "none" ]]; then
    notes="ok(${v2_seen_evidence})"
  fi
  if [[ "${active_source}" != "${expect_source}" ]]; then
    status="FAIL"
    notes="active_source=${active_source}, expect=${expect_source}"
  fi
  if [[ "${expect_v2}" == "yes" && "${v2_seen}" != "yes" ]]; then
    status="FAIL"
    notes="expected v2 messages"
  fi
  if [[ "${expect_v2}" == "no" && "${v2_seen}" == "yes" ]]; then
    status="FAIL"
    notes="unexpected v2 messages"
  fi

  append_result "${scenario}" "${status}" "${active_source}" "${fallback_count}" "${recover_count}" \
    "${v1_count}" "${v2_count}" "${v2_seen}" "${notes}"
}

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
if [[ ! -f "${MISSION_STACK_LAUNCH}" ]]; then
  echo "[ERROR] mission stack launch not found: ${MISSION_STACK_LAUNCH}"
  exit 2
fi

mission_profile
source_workspace
mkdir -p "${OUT_DIR}"
RESULT_TSV="${OUT_DIR}/results.tsv"
SUMMARY_MD="${OUT_DIR}/summary.md"
RESULT_JSON="${OUT_DIR}/results.json"

printf "scenario\tstatus\tactive_source\tfallback_count\trecover_count\tv1_count\tv2_count\tv2_seen\tnotes\n" \
  > "${RESULT_TSV}"

run_scenario \
  "v2_primary" \
  "v2" \
  "yes" \
  planning_enable_pathlimits_v1_publish:=true \
  planning_enable_pathlimits_v2_publish:=true \
  control_enable_pathlimits_v1_subscribe:=true \
  control_enable_pathlimits_v2_subscribe:=true \
  control_prefer_pathlimits_v2:=true \
  control_enable_pathlimits_v2_auto_fallback:=true \
  control_pathlimits_source_stale_timeout_sec:=0.8

run_scenario \
  "v2_fallback_no_v2_publish" \
  "v1-fallback" \
  "no" \
  planning_enable_pathlimits_v1_publish:=true \
  planning_enable_pathlimits_v2_publish:=false \
  control_enable_pathlimits_v1_subscribe:=true \
  control_enable_pathlimits_v2_subscribe:=true \
  control_prefer_pathlimits_v2:=true \
  control_enable_pathlimits_v2_auto_fallback:=true \
  control_pathlimits_source_stale_timeout_sec:=0.8

python3 - "${RESULT_TSV}" "${SUMMARY_MD}" "${RESULT_JSON}" <<'PY'
import csv
import json
import pathlib
import sys

tsv_path = pathlib.Path(sys.argv[1])
summary_path = pathlib.Path(sys.argv[2])
json_path = pathlib.Path(sys.argv[3])

rows = []
with tsv_path.open("r", encoding="utf-8") as f:
    reader = csv.DictReader(f, delimiter="\t")
    for row in reader:
        rows.append(row)

json_path.write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

lines = []
lines.append("# PathLimitsV2 Cutover Replay Acceptance\n")
lines.append("| scenario | status | active_source | fallback_count | recover_count | v1_count | v2_count | v2_seen | notes |")
lines.append("|---|---|---|---:|---:|---:|---:|---|---|")
for row in rows:
    lines.append(
        f"| {row['scenario']} | {row['status']} | {row['active_source']} | "
        f"{row['fallback_count']} | {row['recover_count']} | {row['v1_count']} | "
        f"{row['v2_count']} | {row['v2_seen']} | {row['notes']} |"
    )

summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

echo "[pathlimits-v2] results: ${RESULT_TSV}"
echo "[pathlimits-v2] summary: ${SUMMARY_MD}"
echo "[pathlimits-v2] json: ${RESULT_JSON}"

if awk -F '\t' 'NR>1 && $2 != "PASS" {exit 1}' "${RESULT_TSV}"; then
  echo "[pathlimits-v2] ACCEPTANCE PASS"
else
  echo "[pathlimits-v2] ACCEPTANCE FAIL"
  exit 1
fi
