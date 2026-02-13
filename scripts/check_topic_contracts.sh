#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/noetic/setup.bash
if [[ -f "${ROOT_DIR}/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/devel/setup.bash"
fi

CMD_TOPIC="verify/cmd"
CARSTATE_TOPIC="verify/state"
PATHLIMITS_TOPIC="verify/path"
APPROACH_TOPIC="verify/goal"
LEGACY_CMD_TOPIC="legacy/cmd"
LEGACY_CARSTATE_TOPIC="legacy/state"
LEGACY_APPROACH_TOPIC="legacy/goal"

FAIL=0

check_contains() {
  local file="$1"
  local pattern="$2"
  local label="$3"
  if ! rg -q --fixed-strings "$pattern" "$file"; then
    echo "[FAIL] ${label}: missing '${pattern}'"
    FAIL=1
  fi
}

check_not_contains() {
  local file="$1"
  local pattern="$2"
  local label="$3"
  if rg -q --fixed-strings "$pattern" "$file"; then
    echo "[FAIL] ${label}: should not contain '${pattern}'"
    FAIL=1
  fi
}

run_dump() {
  local launch_file="$1"
  shift
  local out
  out="$(mktemp)"
  roslaunch --dump-params "$launch_file" "$@" > "$out"
  echo "$out"
}

check_standard_mission() {
  local launch_name="$1"
  local launch_file="${ROOT_DIR}/src/fsd_launch/launch/${launch_name}.launch"

  echo "[INFO] checking ${launch_name}.launch"
  local out
  out="$(run_dump "$launch_file" \
    cmd_topic:=${CMD_TOPIC} \
    carstate_topic:=${CARSTATE_TOPIC} \
    pathlimits_topic:=${PATHLIMITS_TOPIC} \
    approaching_goal_topic:=${APPROACH_TOPIC})"

  check_contains "$out" "/control/cmd_topic: ${CMD_TOPIC}" "${launch_name}"
  check_contains "$out" "/control/carstate_topic: ${CARSTATE_TOPIC}" "${launch_name}"
  check_contains "$out" "/control/pathlimits_topic: ${PATHLIMITS_TOPIC}" "${launch_name}"
  check_contains "$out" "/control/approaching_goal_topic: ${APPROACH_TOPIC}" "${launch_name}"

  check_contains "$out" "/location_node/topics/car_state_in: ${CARSTATE_TOPIC}" "${launch_name}"
  check_contains "$out" "/location_node/topics/car_state_out: ${CARSTATE_TOPIC}" "${launch_name}"

  check_contains "$out" "/planning_pipeline/input_pose_topic: ${CARSTATE_TOPIC}" "${launch_name}"
  check_contains "$out" "/planning_pipeline/output_pathlimits_topic: ${PATHLIMITS_TOPIC}" "${launch_name}"
  check_contains "$out" "/planning_pipeline/output_approaching_goal_topic: ${APPROACH_TOPIC}" "${launch_name}"
  check_contains "$out" "/planning_pipeline/topics/car_state: ${CARSTATE_TOPIC}" "${launch_name}"
  check_contains "$out" "/planning_pipeline/topics/pathlimits: ${PATHLIMITS_TOPIC}" "${launch_name}"
  check_contains "$out" "/planning_pipeline/topics/approaching_goal: ${APPROACH_TOPIC}" "${launch_name}"

  check_contains "$out" "/vehicle_interface/cmd_topic: ${CMD_TOPIC}" "${launch_name}"

  check_contains "$out" "/perception/lidar_cluster/lidar_cluster_node/topics/car_state_in: ${CARSTATE_TOPIC}" "${launch_name}"
  check_contains "$out" "/perception/lidar_cluster/lidar_cluster_node/topics/car_state_out: ${CARSTATE_TOPIC}" "${launch_name}"

  rm -f "$out"
}

check_legacy_toggle() {
  local launch_file="${ROOT_DIR}/src/fsd_launch/launch/trackdrive.launch"

  echo "[INFO] checking legacy toggles via trackdrive.launch"
  local out
  out="$(run_dump "$launch_file" \
    cmd_topic:=${CMD_TOPIC} \
    carstate_topic:=${CARSTATE_TOPIC} \
    pathlimits_topic:=${PATHLIMITS_TOPIC} \
    approaching_goal_topic:=${APPROACH_TOPIC} \
    publish_legacy_cmd_topic:=true \
    subscribe_legacy_topics:=true \
    subscribe_legacy_cmd_topic:=true \
    legacy_cmd_topic:=${LEGACY_CMD_TOPIC} \
    legacy_carstate_topic:=${LEGACY_CARSTATE_TOPIC} \
    legacy_approaching_goal_topic:=${LEGACY_APPROACH_TOPIC})"

  check_contains "$out" "/control/publish_legacy_cmd_topic: true" "legacy-toggle"
  check_contains "$out" "/control/subscribe_legacy_topics: true" "legacy-toggle"
  check_contains "$out" "/control/legacy_cmd_topic: ${LEGACY_CMD_TOPIC}" "legacy-toggle"
  check_contains "$out" "/control/legacy_carstate_topic: ${LEGACY_CARSTATE_TOPIC}" "legacy-toggle"
  check_contains "$out" "/control/legacy_approaching_goal_topic: ${LEGACY_APPROACH_TOPIC}" "legacy-toggle"

  check_contains "$out" "/vehicle_interface/subscribe_legacy_cmd_topic: true" "legacy-toggle"
  check_contains "$out" "/vehicle_interface/legacy_cmd_topic: ${LEGACY_CMD_TOPIC}" "legacy-toggle"

  rm -f "$out"
}

check_ebs() {
  local launch_file="${ROOT_DIR}/src/fsd_launch/launch/ebs_test.launch"

  echo "[INFO] checking ebs_test.launch"
  local out
  out="$(run_dump "$launch_file" \
    enable_ebs_control:=true \
    cmd_topic:=${CMD_TOPIC} \
    carstate_topic:=${CARSTATE_TOPIC} \
    pathlimits_topic:=${PATHLIMITS_TOPIC} \
    approaching_goal_topic:=${APPROACH_TOPIC})"

  check_contains "$out" "/control/cmd_topic: ${CMD_TOPIC}" "ebs_test"
  check_contains "$out" "/control/carstate_topic: ${CARSTATE_TOPIC}" "ebs_test"
  check_contains "$out" "/control/pathlimits_topic: ${PATHLIMITS_TOPIC}" "ebs_test"
  check_contains "$out" "/control/approaching_goal_topic: ${APPROACH_TOPIC}" "ebs_test"

  check_contains "$out" "/location_node/topics/car_state_in: ${CARSTATE_TOPIC}" "ebs_test"
  check_contains "$out" "/location_node/topics/car_state_out: ${CARSTATE_TOPIC}" "ebs_test"

  check_contains "$out" "/vehicle_interface/cmd_topic: ${CMD_TOPIC}" "ebs_test"
  check_contains "$out" "/perception/lidar_cluster/lidar_cluster_node/topics/car_state_in: ${CARSTATE_TOPIC}" "ebs_test"
  check_contains "$out" "/perception/lidar_cluster/lidar_cluster_node/topics/car_state_out: ${CARSTATE_TOPIC}" "ebs_test"

  check_not_contains "$out" "/planning_pipeline/" "ebs_test"

  rm -f "$out"
}

check_full_sim() {
  local launch_file="${ROOT_DIR}/src/fsd_launch/launch/simulation/full_simulation.launch"

  echo "[INFO] checking full_simulation.launch (planner:=unified)"
  local out
  out="$(run_dump "$launch_file" \
    planner:=unified \
    cmd_topic:=${CMD_TOPIC} \
    carstate_topic:=${CARSTATE_TOPIC} \
    pathlimits_topic:=${PATHLIMITS_TOPIC} \
    approaching_goal_topic:=${APPROACH_TOPIC})"

  check_contains "$out" "/control/cmd_topic: ${CMD_TOPIC}" "full_sim(unified)"
  check_contains "$out" "/control/carstate_topic: ${CARSTATE_TOPIC}" "full_sim(unified)"
  check_contains "$out" "/control/pathlimits_topic: ${PATHLIMITS_TOPIC}" "full_sim(unified)"
  check_contains "$out" "/control/approaching_goal_topic: ${APPROACH_TOPIC}" "full_sim(unified)"
  check_contains "$out" "/planning_pipeline/input_pose_topic: ${CARSTATE_TOPIC}" "full_sim(unified)"
  check_contains "$out" "/planning_pipeline/output_pathlimits_topic: ${PATHLIMITS_TOPIC}" "full_sim(unified)"
  check_contains "$out" "/planning_pipeline/output_approaching_goal_topic: ${APPROACH_TOPIC}" "full_sim(unified)"
  check_contains "$out" "/perception/lidar_cluster/lidar_cluster_node/topics/car_state_in: ${CARSTATE_TOPIC}" "full_sim(unified)"
  check_contains "$out" "/perception/lidar_cluster/lidar_cluster_node/topics/car_state_out: ${CARSTATE_TOPIC}" "full_sim(unified)"
  check_not_contains "$out" "/vehicle_interface/" "full_sim(unified)"

  rm -f "$out"

  echo "[INFO] checking full_simulation.launch (planner:=none)"
  out="$(run_dump "$launch_file" \
    planner:=none \
    cmd_topic:=${CMD_TOPIC} \
    carstate_topic:=${CARSTATE_TOPIC} \
    pathlimits_topic:=${PATHLIMITS_TOPIC} \
    approaching_goal_topic:=${APPROACH_TOPIC})"

  check_contains "$out" "/control/cmd_topic: ${CMD_TOPIC}" "full_sim(none)"
  check_not_contains "$out" "/planning_pipeline/" "full_sim(none)"

  rm -f "$out"
}

main() {
  check_standard_mission "trackdrive"
  check_standard_mission "autocross"
  check_standard_mission "skidpad"
  check_standard_mission "acceleration"
  check_legacy_toggle
  check_ebs
  check_full_sim

  if [[ "$FAIL" -ne 0 ]]; then
    echo "[RESULT] topic contract check FAILED"
    exit 1
  fi
  echo "[RESULT] topic contract check PASSED"
}

main "$@"
