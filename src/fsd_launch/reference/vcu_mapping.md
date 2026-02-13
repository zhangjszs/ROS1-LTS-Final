# VCU Mapping (Draft)

## Scope
This document defines the current control mission-to-mode mapping and VCU interface placeholders.
It reflects repository state as of 2026-02-09 and must be finalized before real-vehicle EBS tests.

## 1) Mission -> `control_mode` Mapping

| Mission launch | control_mode | Controller selected in `control_node` |
|---|---:|---|
| `trackdrive.launch` | 4 | HighController (`mode != 1,2,3`) |
| `autocross.launch` | 4 | HighController |
| `acceleration.launch` | 2 | LineController |
| `skidpad.launch` | 3 | SkipController |
| `ebs_test.launch` | 5 (placeholder) | HighController (current fallback behavior) |

Reference:
- `src/control_ros/src/control_node.cpp` (`InitController`)
- `src/fsd_launch/launch/*.launch` (`control_mode` arguments)

## 2) Current Command/Status Topics

| Direction | Topic | Message |
|---|---|---|
| Control -> VCU bridge | `vehicle/cmd` (canonical) | `autodrive_msgs/HUAT_VehicleCmd` |
| VCU -> racing number writer | `vehicleStatusMsg` (default) | `autodrive_msgs/HUAT_VehicleStatus` |

Reference:
- `src/control_ros/src/control_node.cpp`
- `src/vehicle_racing_num_ros/src/vehicle_racing_num_node.cpp`

Compatibility notes:
- Legacy command topic alias `vehcileCMDMsg` is optional and disabled by default.
- Enable in control side with `publish_legacy_cmd_topic:=true`.
- Enable in vehicle interface side with `subscribe_legacy_cmd_topic:=true` (if required by legacy bridge).

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
- Do not run `enable_ebs_control:=true` on real vehicle before completing section 4 and bench validation.
