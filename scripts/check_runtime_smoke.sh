#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/noetic/setup.bash
if [[ -f "${ROOT_DIR}/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/devel/setup.bash"
fi

TMP_DIR="$(mktemp -d)"
PASS_COUNT=0
SKIP_COUNT=0
FAIL_COUNT=0

SMOKE_TIMEOUT_SEC="${SMOKE_TIMEOUT_SEC:-45}"
STARTUP_WAIT_SEC="${STARTUP_WAIT_SEC:-30}"
RUNTIME_SMOKE_REQUIRED="${RUNTIME_SMOKE_REQUIRED:-}"

is_truthy() {
  local value="${1:-}"
  case "${value,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

if [[ -z "${RUNTIME_SMOKE_REQUIRED}" ]]; then
  if is_truthy "${CI:-0}"; then
    RUNTIME_SMOKE_REQUIRED=1
  else
    RUNTIME_SMOKE_REQUIRED=0
  fi
fi

cleanup() {
  rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

is_permission_block() {
  local log_file="$1"
  rg -q "PermissionError: \\[Errno 1\\] Operation not permitted|netifaces\\.interfaces" "${log_file}"
}

mark_skip_or_fail_for_permission() {
  local label="$1"
  local log_file="$2"

  if is_truthy "${RUNTIME_SMOKE_REQUIRED}"; then
    echo "[FAIL] ${label}: environment permission block detected and strict mode is enabled"
    sed -n '1,120p' "${log_file}"
    FAIL_COUNT=$((FAIL_COUNT + 1))
  else
    echo "[SKIP] ${label}: environment permission block detected (non-strict mode)"
    SKIP_COUNT=$((SKIP_COUNT + 1))
  fi
}

start_and_check() {
  local label="$1"
  local launch_file="$2"
  shift 2

  local expected_nodes=()
  while [[ "$#" -gt 0 ]]; do
    if [[ "$1" == "--" ]]; then
      shift
      break
    fi
    expected_nodes+=("$1")
    shift
  done
  local launch_args=("$@")

  local log_file="${TMP_DIR}/${label}.log"
  local run_ros_home="${TMP_DIR}/ros_home_${label}"
  local run_ros_log_dir="${TMP_DIR}/roslog_${label}"
  mkdir -p "${run_ros_home}" "${run_ros_log_dir}"
  echo "[INFO] runtime smoke: ${label}"

  ROS_HOME="${run_ros_home}" ROS_LOG_DIR="${run_ros_log_dir}" \
    timeout "${SMOKE_TIMEOUT_SEC}s" roslaunch "${launch_file}" "${launch_args[@]}" >"${log_file}" 2>&1 &
  local launch_pid=$!

  local ok=0
  local permission_block=0
  for _ in $(seq 1 "${STARTUP_WAIT_SEC}"); do
    if [[ -s "${log_file}" ]] && is_permission_block "${log_file}"; then
      permission_block=1
      break
    fi
    if rosnode list >/dev/null 2>&1; then
      local nodes
      nodes="$(rosnode list 2>/dev/null || true)"
      local all_found=1
      for node in "${expected_nodes[@]}"; do
        if ! grep -q "^${node}$" <<<"${nodes}"; then
          all_found=0
          break
        fi
      done
      if [[ "${all_found}" -eq 1 ]]; then
        ok=1
        break
      fi
    fi
    sleep 1
  done

  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true

  if [[ "${permission_block}" -eq 1 ]] || is_permission_block "${log_file}"; then
    mark_skip_or_fail_for_permission "${label}" "${log_file}"
    return
  fi

  if [[ "${ok}" -ne 1 ]]; then
    echo "[FAIL] ${label}: expected nodes not all up"
    sed -n '1,120p' "${log_file}"
    FAIL_COUNT=$((FAIL_COUNT + 1))
    return
  fi

  if rg -q "RLException|PermissionError|Traceback" "${log_file}"; then
    echo "[FAIL] ${label}: launch exception detected"
    sed -n '1,120p' "${log_file}"
    FAIL_COUNT=$((FAIL_COUNT + 1))
    return
  fi

  echo "[PASS] ${label}"
  PASS_COUNT=$((PASS_COUNT + 1))
}

main() {
  start_and_check \
    "trackdrive_headless" \
    "${ROOT_DIR}/src/fsd_launch/launch/trackdrive.launch" \
    "/control" "/planning_pipeline" "/location_node" "/perception/lidar_cluster/lidar_cluster_node" "/vehicle_interface" \
    -- \
    launch_rviz:=false launch_viz:=false simulation:=false

  start_and_check \
    "ebs_headless" \
    "${ROOT_DIR}/src/fsd_launch/launch/ebs_test.launch" \
    "/control" "/planning_disabled_notice" "/location_node" "/vehicle_interface" \
    -- \
    enable_ebs_control:=true launch_rviz:=false launch_viz:=false simulation:=false

  start_and_check \
    "simulation_node_only" \
    "${ROOT_DIR}/src/simulation_ros/launch/simulation.launch" \
    "/simulation_node" \
    -- \
    publish_tf:=false

  echo "[INFO] runtime smoke summary: pass=${PASS_COUNT}, skip=${SKIP_COUNT}, fail=${FAIL_COUNT}, required=${RUNTIME_SMOKE_REQUIRED}"

  if [[ "${FAIL_COUNT}" -ne 0 ]]; then
    echo "[RESULT] runtime smoke check FAILED"
    exit 1
  fi
  if [[ "${SKIP_COUNT}" -ne 0 ]]; then
    echo "[RESULT] runtime smoke check SKIPPED (environment-limited)"
    exit 0
  fi
  echo "[RESULT] runtime smoke check PASSED"
}

main "$@"
