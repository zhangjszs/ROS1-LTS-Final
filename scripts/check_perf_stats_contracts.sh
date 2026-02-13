#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/noetic/setup.bash
if [[ -f "${ROOT_DIR}/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/devel/setup.bash"
fi

FAIL=0

run_perf_test() {
  local package="$1"
  local target="$2"
  local binary="${ROOT_DIR}/devel/.private/${package}/lib/${package}/${target}"
  local build_dir="${ROOT_DIR}/build/${package}"

  if [[ ! -d "${build_dir}" ]]; then
    echo "[INFO] build directory missing for ${package}, running catkin build ${package}"
    if ! catkin build --no-status --summarize "${package}" >/dev/null; then
      echo "[FAIL] failed to build package ${package}"
      FAIL=1
      return
    fi
  fi

  echo "[INFO] building test target ${target}"
  if ! make -C "${build_dir}" "${target}" -j4 >/dev/null; then
    echo "[FAIL] failed to build test target ${target}"
    FAIL=1
    return
  fi

  if [[ ! -x "${binary}" ]]; then
    echo "[FAIL] missing test binary: ${binary}"
    FAIL=1
    return
  fi

  echo "[INFO] running ${target}"
  if ! "${binary}" --gtest_color=no; then
    echo "[FAIL] test execution failed: ${target}"
    FAIL=1
    return
  fi
}

main() {
  run_perf_test "localization_ros" "localization_ros_perf_stats_test"
  run_perf_test "perception_ros" "perception_ros_perf_stats_test"

  if [[ "${FAIL}" -ne 0 ]]; then
    echo "[RESULT] perf stats contract check FAILED"
    exit 1
  fi
  echo "[RESULT] perf stats contract check PASSED"
}

main "$@"
