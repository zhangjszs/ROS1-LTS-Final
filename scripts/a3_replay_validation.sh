#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULT_DIR="${RESULT_DIR:-/tmp/a3_validation}"
BAG_PATH="${1:-/home/kerwin/rosbag/track.bag}"
PLAY_RATE="${2:-2.0}"
TIMEOUT_S="${TIMEOUT_S:-95}"

mkdir -p "${RESULT_DIR}" /tmp/roslog
rm -f "${RESULT_DIR}/launch.log" \
      "${RESULT_DIR}/control_diag.txt" \
      "${RESULT_DIR}/localization_diag.txt" \
      "${RESULT_DIR}/planning_diag.txt" \
      "${RESULT_DIR}/global_diag.txt" \
      "${RESULT_DIR}/global_diag_agg.txt" \
      "${RESULT_DIR}/vehicle_cmd.txt"

source "${ROOT_DIR}/devel/setup.bash"
export ROS_LOG_DIR=/tmp/roslog

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[A3] Start one replay validation"
echo "[A3] bag=${BAG_PATH} rate=${PLAY_RATE} result_dir=${RESULT_DIR}"

timeout "${TIMEOUT_S}s" roslaunch fsd_launch trackdrive.launch \
  simulation:=true \
  bag:="${BAG_PATH}" \
  rate:="${PLAY_RATE}" \
  loop:=false \
  launch_rviz:=false \
  launch_viz:=false \
  > "${RESULT_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 30); do
  if rostopic list >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

timeout 20s rostopic echo -n 1 /control/diagnostics > "${RESULT_DIR}/control_diag.txt" 2>/dev/null || true
timeout 20s rostopic echo -n 20 /localization/diagnostics > "${RESULT_DIR}/localization_diag.txt" 2>/dev/null || true
timeout 20s rostopic echo -n 20 /planning/diagnostics > "${RESULT_DIR}/planning_diag.txt" 2>/dev/null || true
timeout 20s rostopic echo -n 20 /diagnostics > "${RESULT_DIR}/global_diag.txt" 2>/dev/null || true
timeout 20s rostopic echo -n 20 /diagnostics_agg > "${RESULT_DIR}/global_diag_agg.txt" 2>/dev/null || true
timeout 20s rostopic echo -n 10 /vehcileCMDMsg > "${RESULT_DIR}/vehicle_cmd.txt" 2>/dev/null || true

wait "${LAUNCH_PID}" || true
unset LAUNCH_PID

pass=1

check_kv_in_file() {
  local file="$1"
  local key="$2"
  local expected="$3"
  local description="$4"
  if rg -q -U "key: \"${key}\"\\n\\s*value: \"${expected}\"" "${file}"; then
    echo "[PASS] ${description}"
  else
    echo "[FAIL] ${description}"
    pass=0
  fi
}

check_status_name_in_file() {
  local file="$1"
  local expected="$2"
  local description="$3"
  if rg -q "name: \"${expected}\"" "${file}"; then
    echo "[PASS] ${description}"
  else
    echo "[FAIL] ${description}"
    pass=0
  fi
}

check_kv() {
  local key="$1"
  local expected="$2"
  local description="$3"
  check_kv_in_file "${RESULT_DIR}/control_diag.txt" "${key}" "${expected}" "${description}"
}

if [[ ! -s "${RESULT_DIR}/control_diag.txt" ]]; then
  echo "[FAIL] /control/diagnostics not captured"
  pass=0
else
  check_kv "mode_source" "param" "mode_source=param"
  check_kv "file_mode_fallback_used" "false" "file_mode_fallback_used=false"
  check_kv "external_stop_source" "disabled" "external_stop_source=disabled in sim replay"
fi

if [[ ! -s "${RESULT_DIR}/localization_diag.txt" ]]; then
  echo "[FAIL] /localization/diagnostics not captured"
  pass=0
else
  check_status_name_in_file "${RESULT_DIR}/localization_diag.txt" "localization_entry_health" "/localization/diagnostics contains localization_entry_health"
fi

if [[ ! -s "${RESULT_DIR}/planning_diag.txt" ]]; then
  echo "[FAIL] /planning/diagnostics not captured"
  pass=0
else
  check_status_name_in_file "${RESULT_DIR}/planning_diag.txt" "planning_entry_health" "/planning/diagnostics contains planning_entry_health"
fi

if [[ ! -s "${RESULT_DIR}/global_diag.txt" ]]; then
  echo "[FAIL] /diagnostics not captured"
  pass=0
else
  check_status_name_in_file "${RESULT_DIR}/global_diag.txt" "control_entry_health" "/diagnostics contains control_entry_health"
  check_status_name_in_file "${RESULT_DIR}/global_diag.txt" "localization_entry_health" "/diagnostics contains localization_entry_health"
  check_status_name_in_file "${RESULT_DIR}/global_diag.txt" "planning_entry_health" "/diagnostics contains planning_entry_health"
  check_kv_in_file "${RESULT_DIR}/global_diag.txt" "mode_source" "param" "/diagnostics mode_source=param"
fi

if [[ ! -s "${RESULT_DIR}/global_diag_agg.txt" ]]; then
  echo "[FAIL] /diagnostics_agg not captured"
  pass=0
else
  if rg -q 'name: ".*ControlEntry.*control_entry_health"' "${RESULT_DIR}/global_diag_agg.txt"; then
    echo "[PASS] /diagnostics_agg grouped ControlEntry/control_entry_health"
  else
    echo "[FAIL] /diagnostics_agg missing grouped ControlEntry/control_entry_health"
    pass=0
  fi
  if rg -q 'name: ".*LocalizationEntry.*localization_entry_health"' "${RESULT_DIR}/global_diag_agg.txt"; then
    echo "[PASS] /diagnostics_agg grouped LocalizationEntry/localization_entry_health"
  else
    echo "[FAIL] /diagnostics_agg missing grouped LocalizationEntry/localization_entry_health"
    pass=0
  fi
  if rg -q 'name: ".*PlanningEntry.*planning_entry_health"' "${RESULT_DIR}/global_diag_agg.txt"; then
    echo "[PASS] /diagnostics_agg grouped PlanningEntry/planning_entry_health"
  else
    echo "[FAIL] /diagnostics_agg missing grouped PlanningEntry/planning_entry_health"
    pass=0
  fi
fi

cmd_count=$(rg -c "head1:" "${RESULT_DIR}/vehicle_cmd.txt" || true)
if [[ "${cmd_count}" -ge 1 ]]; then
  echo "[PASS] /vehcileCMDMsg published (${cmd_count} samples)"
else
  echo "[FAIL] /vehcileCMDMsg not published"
  pass=0
fi

echo "[A3] artifacts:"
echo "  - ${RESULT_DIR}/launch.log"
echo "  - ${RESULT_DIR}/control_diag.txt"
echo "  - ${RESULT_DIR}/localization_diag.txt"
echo "  - ${RESULT_DIR}/planning_diag.txt"
echo "  - ${RESULT_DIR}/global_diag.txt"
echo "  - ${RESULT_DIR}/global_diag_agg.txt"
echo "  - ${RESULT_DIR}/vehicle_cmd.txt"

if [[ "${pass}" -ne 1 ]]; then
  echo "[A3] validation failed"
  exit 1
fi

echo "[A3] validation passed"
