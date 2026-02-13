#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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

check_common_legacy_defaults() {
  local file="$1"
  local label="$2"
  check_contains "$file" '<arg name="publish_legacy_cmd_topic" default="false"' "${label}"
  check_contains "$file" '<arg name="subscribe_legacy_topics" default="false"' "${label}"
}

check_vehicle_legacy_defaults() {
  local file="$1"
  local label="$2"
  check_contains "$file" '<arg name="subscribe_legacy_cmd_topic" default="false"' "${label}"
}

main() {
  local missions=(
    "${ROOT_DIR}/src/fsd_launch/launch/trackdrive.launch"
    "${ROOT_DIR}/src/fsd_launch/launch/autocross.launch"
    "${ROOT_DIR}/src/fsd_launch/launch/skidpad.launch"
    "${ROOT_DIR}/src/fsd_launch/launch/acceleration.launch"
    "${ROOT_DIR}/src/fsd_launch/launch/ebs_test.launch"
  )

  for mission in "${missions[@]}"; do
    echo "[INFO] checking mission deprecation defaults: ${mission##*/}"
    check_common_legacy_defaults "$mission" "${mission##*/}"
    check_vehicle_legacy_defaults "$mission" "${mission##*/}"
  done

  local full_sim="${ROOT_DIR}/src/fsd_launch/launch/simulation/full_simulation.launch"
  echo "[INFO] checking mission deprecation defaults: ${full_sim##*/}"
  check_common_legacy_defaults "$full_sim" "${full_sim##*/}"

  local mission_stack="${ROOT_DIR}/src/fsd_launch/launch/subsystems/mission_stack.launch"
  echo "[INFO] checking mission stack compatibility defaults"
  check_contains "$mission_stack" '<arg name="publish_legacy_cmd_topic" default="false"' "mission_stack"
  check_contains "$mission_stack" '<arg name="subscribe_legacy_topics" default="false"' "mission_stack"
  check_contains "$mission_stack" '<arg name="subscribe_legacy_cmd_topic" default="false"' "mission_stack"

  local control_subsystem="${ROOT_DIR}/src/fsd_launch/launch/subsystems/control.launch"
  local control_entry="${ROOT_DIR}/src/control_ros/launch/controler.launch"
  echo "[INFO] checking control file-fallback defaults"
  check_contains "$control_subsystem" '<arg name="enable_file_mode_fallback" default="false"' "control_subsystem"
  check_contains "$control_subsystem" '<arg name="enable_external_stop_file" default="false"' "control_subsystem"
  check_contains "$control_entry" '<arg name="enable_file_mode_fallback" default="false"' "control_entry"
  check_contains "$control_entry" '<arg name="enable_external_stop_file" default="false"' "control_entry"

  local viz_launch="${ROOT_DIR}/src/fsd_visualization/launch/visualization.launch"
  echo "[INFO] checking visualization legacy path compatibility defaults"
  check_contains "$viz_launch" '<arg name="enable_legacy_partial_full" default="false"' "visualization"
  check_contains "$viz_launch" '<param name="compat/enable_legacy_partial_full" value="$(arg enable_legacy_partial_full)"' "visualization"

  local stage2_missions=(
    "${ROOT_DIR}/src/fsd_launch/launch/trackdrive.launch"
    "${ROOT_DIR}/src/fsd_launch/launch/autocross.launch"
    "${ROOT_DIR}/src/fsd_launch/launch/skidpad.launch"
    "${ROOT_DIR}/src/fsd_launch/launch/acceleration.launch"
    "${ROOT_DIR}/src/fsd_launch/launch/ebs_test.launch"
  )
  echo "[INFO] checking mission launches are wired via mission_stack"
  for mission in "${stage2_missions[@]}"; do
    check_contains "$mission" 'subsystems/mission_stack.launch' "${mission##*/}"
    check_not_contains "$mission" 'subsystems/perception.launch' "${mission##*/}"
    check_not_contains "$mission" 'subsystems/localization.launch' "${mission##*/}"
    check_not_contains "$mission" 'subsystems/planning.launch' "${mission##*/}"
    check_not_contains "$mission" 'subsystems/control.launch' "${mission##*/}"
    check_not_contains "$mission" 'subsystems/vehicle.launch' "${mission##*/}"
  done

  if [[ "$FAIL" -ne 0 ]]; then
    echo "[RESULT] deprecation contract check FAILED"
    exit 1
  fi

  echo "[RESULT] deprecation contract check PASSED"
}

main "$@"
