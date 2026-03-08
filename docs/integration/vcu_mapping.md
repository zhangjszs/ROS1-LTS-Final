# VCU Mapping (Draft)

## Scope
This document records the current repository state for mission-to-control-mode mapping and VCU interface placeholders.
It was reconciled against code on 2026-03-06 and still requires protocol finalization before any real-vehicle EBS validation.

## 1) Mission -> `control_mode` Mapping

| Mission launch | current `control_mode` | controller selected in `control_node` | status |
|---|---:|---|---|
| `trackdrive.launch` | 4 | `HighController` | current code path |
| `autocross.launch` | 4 | `HighController` | current code path |
| `acceleration.launch` | 1 | `TestController` | current code path; needs control/protocol re-check |
| `skidpad.launch` | 3 | `SkipController` | current code path |
| `ebs_test.launch` | 5 | `HighController` | placeholder / fallback behavior |

Reference:
- `src/control_ros/src/control_node.cpp` (`InitController`)
- `src/fsd_common/include/fsd_common/control_mode.hpp`
- `src/fsd_launch/launch/*.launch` (`control_mode` arguments)

## 2) Current Command/Status Topics

| Direction | Topic | Message | status |
|---|---|---|---|
| Control -> VCU bridge | `vehicle/cmd` | `autodrive_msgs/HUAT_VehicleCmd` | canonical |
| VCU -> racing number writer | `vehicle/status` | `autodrive_msgs/HUAT_VehicleStatus` | canonical |

Reference:
- `src/fsd_common/include/fsd_common/topic_contract.hpp`
- `src/vehicle_racing_num_ros/src/vehicle_racing_num_node.cpp`

Compatibility notes:
- Older documents may still mention `vehcileCMDMsg` or `vehicleStatusMsg`; they are not the canonical defaults anymore.
- If a legacy bridge still exists outside this repository, document its remap explicitly instead of treating legacy names as repository truth.

## 3) Message Field Map (Current)

### 3.1 Outbound control command (`HUAT_VehicleCmd`)
- `steering`
- `pedal_ratio`
- `brake_force`
- `gear_position`
- `working_mode`
- `racing_num`
- `racing_status`
- `checksum`

### 3.2 Inbound vehicle status (`HUAT_VehicleStatus`)
- `work_mode`
- `racing_num`
- `fault_type`
- wheel speeds (`speed_*`)

## 4) EBS-specific TODO (Must finalize before real vehicle)

- `TODO_VCU_CMD_TOPIC`
- `TODO_VCU_STATUS_TOPIC`
- `TODO_VCU_BRAKE_CMD_FIELD`
- `TODO_VCU_EBS_ARM_FIELD`
- `TODO_VCU_EBS_TRIGGER_CONDITION`
- `TODO_VCU_HEARTBEAT_TIMEOUT_MS`
- `TODO_VCU_FAILSAFE_BEHAVIOR`

## 5) Safety Notes

- `control_mode=5` is currently an interface placeholder and does not select a dedicated EBS controller yet.
- `ebs_test.launch` still disables planning by design (`planner:=none`).
- `acceleration.launch` currently uses `control_mode=1 -> TestController`; confirm with control logic owners whether this is the intended acceleration mapping or a historical leftover.
- Do not run `enable_ebs_control:=true` on a real vehicle before completing section 4 and bench validation.
