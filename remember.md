# remember.md

## 1) 当前阶段定位
- 当前以 `rosbag` 回放为主，`EBS` 先保留接口，不做实车制动闭环。
- 车辆固定为 `FSAC A13`，暂不做多车型分支。
- LiDAR/IMU 外参旋转先按 `0,0,0`（roll/pitch/yaw）处理。

## 2) 当前确认参数（可直接沿用）

### 2.1 控制参数（`src/control_ros/config/param.yaml`）
```yaml
car_arg:
  length: 1.55
  front_axle: 0.5
  rear_axle: 1.05
  delta_max: 0.4
  delta_min: 0.0174
  cg_to_front: 0.77
  cg_to_rear: 0.78
  mass: 190.0
```

### 2.2 车体几何参数（`src/localization_ros/config/location_common.yaml`）
```yaml
length:
  lidarToIMUDist: 1.87
  frontToIMUdistanceX: 0.0
  frontToIMUdistanceY: 0.0
  frontToIMUdistanceZ: 0.0
  rearToIMUdistanceX: 0.0
  rearToIMUdistanceY: 0.0
  rearToIMUdistanceZ: 0.0
```

## 3) EBS 接口现状
- 新增任务入口：`src/fsd_launch/launch/ebs_test.launch`
- 规划侧预留：`planner:=none`（`src/fsd_launch/launch/subsystems/planning.launch`）
- 控制模式预留：`control_mode:=5`（默认不启用控制，需显式 `enable_ebs_control:=true`）

建议测试命令：
```bash
# 仅感知/定位链路验证
roslaunch fsd_launch ebs_test.launch simulation:=true bag:=/path/to.bag

# 预留控制接口验证（仍为占位流程）
roslaunch fsd_launch ebs_test.launch simulation:=true bag:=/path/to.bag enable_ebs_control:=true control_mode:=5
```

## 4) 上实车前必须替换的占位项（TODO）

### 4.1 VCU 协议与通道（必须确认）
- `TODO_VCU_CMD_TOPIC`
- `TODO_VCU_STATUS_TOPIC`
- `TODO_VCU_BRAKE_CMD_FIELD`
- `TODO_VCU_EBS_ARM_FIELD`
- `TODO_VCU_EBS_TRIGGER_CONDITION`
- `TODO_VCU_HEARTBEAT_TIMEOUT_MS`
- `TODO_VCU_FAILSAFE_BEHAVIOR`

### 4.2 任务与控制模式映射（需和车端协议统一）
- `trackdrive -> control_mode=4`
- `accel -> control_mode=2`
- `skidpad -> control_mode=3`
- `ebs_test -> control_mode=5 (placeholder)`

### 4.3 外参与安装标定（A13）
- LiDAR/IMU 旋转目前为 0，仅用于 bag 阶段。
- 实车需替换为标定结果：
  - `TODO_LIDAR_TO_IMU_ROLL`
  - `TODO_LIDAR_TO_IMU_PITCH`
  - `TODO_LIDAR_TO_IMU_YAW`
  - `TODO_LIDAR_TO_IMU_TX`
  - `TODO_LIDAR_TO_IMU_TY`
  - `TODO_LIDAR_TO_IMU_TZ`

## 5) 实车切换前检查清单
- 完成 VCU 字段映射与联调文档。
- 在低速台架验证 `enable_ebs_control:=true` 的完整链路。
- 校验 `control_mode=5` 的状态机不会误触发非 EBS 行为。
- 完成 A13 外参替换并复测定位误差。
- 增加 EBS 场景回归包（触发/不触发/超时/断链）。
