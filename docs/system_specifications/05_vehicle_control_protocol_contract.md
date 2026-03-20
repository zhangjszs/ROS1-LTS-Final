# 车辆控制协议契约（VCU / EBS）

> 目的：明确当前仓库内控制模式、ROS 消息与 UDP 字节帧之间的映射关系，减少车端联调歧义。  
> 范围：仅描述“代码已实现并可验证”的部分；车端协议最终口径以联调结论为准。

---

## 1. 控制模式映射（代码现状）

| 场景入口 | launch 默认 `control_mode` | 控制节点行为 | 下发 `HUAT_VehicleCmd.working_mode` |
|---|---:|---|---:|
| `acceleration.launch` | `2` | `LineController` | `1`（autonomous） |
| `skidpad.launch` | `3` | `SkipController` | `1`（autonomous） |
| `trackdrive.launch` / `autocross.launch` | `4` | `HighController` | `1`（autonomous） |
| `ebs_test.launch` | `5` | `EbsController` 紧急制动输出 | `2`（EBS） |

实现锚点：
- `src/fsd_common/include/fsd_common/control_mode.hpp`
- `src/control_ros/src/control_node.cpp`
- `src/fsd_launch/launch/{acceleration,skidpad,trackdrive,autocross,ebs_test}.launch`

---

## 2. IPC2VCU 发送帧契约（ROS -> UDP）

当前 `vehicle_interface_ros` 按 12 字节发送：

| 字节偏移 | 字段 | 来源 |
|---:|---|---|
| 0 | `head1` | 固定 `0xAA` |
| 1 | `head2` | 固定 `0x55` |
| 2 | `length` | 固定 `12` |
| 3 | `steering` | `HUAT_VehicleCmd.steering` |
| 4 | `brake_force` | `HUAT_VehicleCmd.brake_force` |
| 5 | `pedal_ratio` | `HUAT_VehicleCmd.pedal_ratio` |
| 6 | `gear_position` | `HUAT_VehicleCmd.gear_position` |
| 7 | `working_mode` | `HUAT_VehicleCmd.working_mode` |
| 8 | `racing_num` | `HUAT_VehicleCmd.racing_num` |
| 9 | `racing_status` | `HUAT_VehicleCmd.racing_status` |
| 10 | `checksum_low` | 前 10 字节求和的低 8 位 |
| 11 | `checksum_high` | 前 10 字节求和的高 8 位 |

校验和规则：
- `checksum = sum(vehicle_tx_msg[0..9])`
- 小端写入：`[10]=low8, [11]=high8`

---

## 3. VCU2IPC 接收帧契约（UDP -> ROS）

`publishVehicle()` 当前按如下字节位解析：

| 字节偏移 | 字段 | 输出 ROS 字段 |
|---:|---|---|
| 0..2 | `head1/head2/length` | `HUAT_VehicleStatus.head1/head2/length` |
| 3 | `steering` | `HUAT_VehicleStatus.steering` |
| 4 | `brake_status` | `HUAT_VehicleStatus.brake_status` |
| 5 | `pedal_ratio` | `HUAT_VehicleStatus.pedal_ratio` |
| 6 | `gear_position` | `HUAT_VehicleStatus.gear_position` |
| 7..14 | 四轮速 | `speed_left_front/right_front/left_rear/right_rear` |
| 15 | `command` | `HUAT_VehicleStatus.command` |
| 16 | `work_mode` | `HUAT_VehicleStatus.work_mode` |
| 17 | `racing_num` | `HUAT_VehicleStatus.racing_num` |
| 18 | `fault_type` | `HUAT_VehicleStatus.fault_type` |
| 19..20 | `checksum` | `HUAT_VehicleStatus.checksum` |

---

## 4. 超时 failsafe 契约

`vehicle_interface_ros` 若超过 `CMD_TIMEOUT_SEC=0.1s` 未收到新命令，将立即发送 failsafe 帧：
- `steering=110`
- `brake_force=100`
- `pedal_ratio=0`
- `gear_position=0`
- `working_mode=2`（EBS）
- `racing_status=3`

用途：当上游控制链断链时，强制进入制动模式。

---

## 5. 尚待联调确认（未完成）

1. 车端对 `working_mode`、`racing_status` 的最终语义表是否与当前实现完全一致。
2. `fault_type` 编码与诊断分级映射是否需要扩展文档。
3. EBS 触发/解除条件、心跳超时阈值是否需要车端与上位机双边确认。
4. `enable_ebs_control:=true` 的台架回归场景（触发/不触发/超时/断链）是否覆盖到位。
