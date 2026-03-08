#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${OUT_DIR:-${ROOT_DIR}/perf_reports/data/adapter_replay_$(date +%Y%m%d_%H%M%S)}"
LEGACY_BUDGET_SEC="${LEGACY_BUDGET_SEC:-0.15}"
DEFAULT_SCENARIOS=(
  raw_only
  same_stamp_normal_merge
  late_fused_within_budget
  late_fused_over_deadline
  intermittent_drop_count_mismatch
  duplicate_stale_finalize
  callback_timer_pressure
)
if [[ "$#" -gt 0 ]]; then
  SCENARIOS=("$@")
else
  SCENARIOS=("${DEFAULT_SCENARIOS[@]}")
fi

mkdir -p "${OUT_DIR}"

source_workspace() {
  source /opt/ros/noetic/setup.bash
  if [[ -f "${ROOT_DIR}/devel/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${ROOT_DIR}/devel/setup.bash"
  elif [[ -f "${ROOT_DIR}/devel/setup.sh" ]]; then
    # shellcheck source=/dev/null
    source "${ROOT_DIR}/devel/setup.sh"
  fi
  if [[ -f "${ROOT_DIR}/devel/.private/autodrive_msgs/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${ROOT_DIR}/devel/.private/autodrive_msgs/setup.bash"
  fi
  if [[ -f "${ROOT_DIR}/devel/.private/perception_ros/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${ROOT_DIR}/devel/.private/perception_ros/setup.bash"
  fi
}

source_workspace
export PYTHONPATH="${ROOT_DIR}/devel/.private/autodrive_msgs/lib/python3/dist-packages:${ROOT_DIR}/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-}"

cleanup() {
  if [[ -n "${PUBLISH_PID:-}" ]]; then
    kill "${PUBLISH_PID}" 2>/dev/null || true
    wait "${PUBLISH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${REC_PID:-}" ]]; then
    kill "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

run_one() {
  local scenario="$1"
  local scenario_dir="${OUT_DIR}/${scenario}"
  local launch_log="${scenario_dir}/launch.log"
  local record_log="${scenario_dir}/record.log"
  local publish_log="${scenario_dir}/publish.log"
  local bag_path="${scenario_dir}/${scenario}_results.bag"
  local master_port="$((20000 + RANDOM % 20000))"
  mkdir -p "${scenario_dir}"
  rm -f "${launch_log}" "${record_log}" "${publish_log}" "${bag_path}" "${bag_path}.active"

  export ROS_LOG_DIR="${scenario_dir}/roslog"
  export ROS_MASTER_URI="http://127.0.0.1:${master_port}"
  export ROS_HOSTNAME="127.0.0.1"
  mkdir -p "${ROS_LOG_DIR}"

  timeout 20s roslaunch "${ROOT_DIR}/src/perception_ros/launch/cone_detection_adapter_only.launch" \
    legacy_budget_sec:="${LEGACY_BUDGET_SEC}" \
    > "${launch_log}" 2>&1 &
  LAUNCH_PID=$!

  for _ in $(seq 1 40); do
    if rostopic list >/dev/null 2>&1; then
      break
    fi
    sleep 0.25
  done

  timeout 10s rostopic echo -n 1 /perception/decision/trace >/dev/null 2>&1 || true

  rosbag record -O "${bag_path}" \
    /perception/lidar_cluster/detections \
    /perception/fusion/detections \
    /perception/decision/detections \
    /perception/decision/trace \
    > "${record_log}" 2>&1 &
  REC_PID=$!

  (
    export PYTHONPATH="${PYTHONPATH}"
    python3 "${ROOT_DIR}/scripts/adapter_replay_publisher.py" --scenario "${scenario}"
  ) > "${publish_log}" 2>&1 &
  PUBLISH_PID=$!
  wait "${PUBLISH_PID}"
  unset PUBLISH_PID

  sleep 1.0
  kill "${REC_PID}" 2>/dev/null || true
  wait "${REC_PID}" 2>/dev/null || true
  unset REC_PID

  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" || true
  unset LAUNCH_PID
}

for scenario in "${SCENARIOS[@]}"; do
  echo "[adapter-replay] running ${scenario}"
  run_one "${scenario}"
done

bag_paths=()
for scenario in "${SCENARIOS[@]}"; do
  bag_paths+=("${OUT_DIR}/${scenario}/${scenario}_results.bag")
done
python3 "${ROOT_DIR}/scripts/analyze_adapter_replay.py" --out-dir "${OUT_DIR}" "${bag_paths[@]}"

echo "[adapter-replay] artifacts in ${OUT_DIR}"
echo "[adapter-replay] summary markdown: ${OUT_DIR}/summary.md"
