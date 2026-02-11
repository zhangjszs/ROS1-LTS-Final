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

## 6) 视觉颜色模块暂缓决策（2026-02-11）
- 决策：当前阶段先跳过相机颜色融合，只保留 LiDAR 主链路（几何+置信度），继续推进规划/控制稳定性与速度策略。
- 原因：现阶段闭环主风险不在颜色识别，而在路径与速度策略收敛；先保证 `P1/P2` 主链路稳定。

### 6.1 对后续优化的影响评估
- 影响等级：`中等偏高`（不是阻断项，但会限制上限）。
- 短期（可接受）：
  - 高必 `SAFE_LAP/FAST_LAP`、速度剖面、控制不断流可继续推进。
  - 直线/八字基础闭环不依赖颜色也可跑通。
- 中长期（必须补齐）：
  - 边界语义不完整，颜色相关创新点和鲁棒性收益无法兑现。
  - 复杂工况（错锥/堆叠/遮挡）下边界分配误差会更高，影响圈速上限与稳定性。
  - 定位因子图中的颜色权重项当前仍应保持关闭（`w_color=0.0`）。

### 6.2 补救与接入顺序（后续执行）
- `TODO_COLOR_FUSION_NODE`：新增 `cone_color_fusion_node`，输入 `HUAT_ConeDetections + Image + CameraInfo`，输出 `detections_fused`。
- `TODO_COLOR_TOPIC_SWITCH`：定位输入话题切换到 `perception/lidar_cluster/detections_fused`（保留原话题回退开关）。
- `TODO_COLOR_WEIGHT_RAMP`：`w_color` 从 `0.0` 逐步升至 `0.2~0.4`，每步做 rosbag 回归。
- `TODO_COLOR_FAILSAFE`：相机掉线/低置信度时自动回退 `NONE` 与 LiDAR-only 流程，不中断主链路。

## 7) Planning 参数调优记录（先冻结，后续再调）

> 决策：从现在开始先做“代码架构收敛”，暂停继续调参。  
> 参数先按当前可跑通版本冻结，等架构完成后统一二次标定。

### 7.1 高速规划当前冻结参数（`src/planning_ros/config/high_speed_tracking.yml`）
- 回环链路：
  - `max_dist_loop_closure: 2.3`
  - `loop_close_debounce_frames: 1`
  - `loop_open_debounce_frames: 15`
  - `loop_allow_reopen: false`
  - `enable_loop_fallback_by_lap_counter: true`
  - `loop_fallback_min_laps: 2`（延后兜底触发，优先几何回环）
  - `loop_diag_enable: false`
- 曲率/速度与稳定性：
  - `curvature_limit: 0.222`（速度约束基准）
  - `curvature_warn_limit_scale: 1.08`（仅告警/质量判定容差）
  - `curvature_smoothing_enable: true`
  - `hold_last_valid_path: true`
  - `short_path_stable_relax_points: 3`（已有稳定路径时短路径阈值放宽）

### 7.2 当前回放效果（track bag, rate=2.0）
- `Curvature exceeds*`：`22 -> 1`
- `GENERAL FAILSAFE ACTIVATED`：`0 -> 0`
- `Holding last stable path`：`32 -> 1`
- `fallback activated`：`0`（由几何回环先触发 full）

### 7.3 后续调参待办（架构完成后再开）
- `TODO_TUNE_CURVATURE_WARN_LIMIT_SCALE`
- `TODO_TUNE_SHORT_PATH_STABLE_RELAX`
- `TODO_TUNE_SEARCH_ADAPTIVE_LEVELS`
- `TODO_TUNE_PHASE_SPEED_CAPS`

## 8) 架构优先剩余任务（不含调参）

### 8.1 控制输入收敛 ✅ 已完成
- 控制层收敛为单一 `HUAT_PathLimits` 参数化输入。
- `control_node.cpp`：通过 `~pathlimits_topic` 参数订阅单一话题，去除 mode 硬编码分支。
- `controler.launch`：新增 `pathlimits_topic` arg，由上层 `control.launch` 按 mode 自动映射。
- `control.launch`：`$(eval ...)` 按 mode 映射 → `planning/line_detection/pathlimits` | `planning/skidpad/pathlimits` | `planning/high_speed_tracking/pathlimits`。
- `high_speed_tracking/main.cpp`：partial/full 合并为单一 `planning/high_speed_tracking/pathlimits` 话题。

### 8.2 规划输出接口统一 ✅ 已完成
- `line_detection_node`：移除 `nav_msgs::Path` 发布（`path_pub_`、`PublishPath()`）。
- `skidpad_detection_node`：移除 `nav_msgs::Path` 发布（`log_path_pub_`、`PublishPath()`）。
- 三赛项统一以 `HUAT_PathLimits` 为唯一对控制有效输出。

### 8.3 速度规划代码复用收敛 ✅ 已完成（line/skidpad/high_speed）
- 新增 `planning_core/include/planning_core/speed_profile.hpp`（header-only 共享模块）。
- 提供 `ComputeCurvatures()` 和 `ComputeSpeedProfile()`，统一曲率计算与速度剖面生成。
- `line_detection_node` 和 `skidpad_detection_node` 已迁移到共享模块。
- `WayComputer::fillPathDynamics` 速度剖面部分已迁移到共享 `ComputeSpeedProfile`。
  - 自适应曲率平滑（多阶段 retry + spatial blend）保留为 WayComputer 特有逻辑。
  - 曲率计算保留 WayComputer 内部版本（含 3-point moving average + violation counting，供自适应平滑循环使用）。
  - mode-blended speed cap（SAFE_LAP/FAST_LAP 渐进过渡）在调用共享模块前计算好传入。

### 8.4 统一管线重构（M5 对齐）— ✅ 已完成
- 目标：从"三节点并行演化"收敛到 mission-aware 统一 planning pipeline。
- 当前已完成的基础设施：
  - 控制侧：单一 `pathlimits_topic` 输入，mission 通过 launch arg 驱动。
  - 规划侧：`HUAT_PathLimits` 为唯一输出接口，`speed_profile.hpp` 为共享速度规划。
- 统一管线阶段设计（S1→S2→S3→S4）：
  - **S1 感知输入**：`HUAT_ConeDetections` + `HUAT_CarState`（已统一）
  - **S2 路径生成**：mission-config 驱动选择算法后端（line/skidpad/high_speed）
  - **S3 速度规划**：`planning_core::ComputeSpeedProfile` 统一入口
  - **S4 输出**：`HUAT_PathLimits` 单话题输出到控制层
- ~~`TODO_PLANNING_PIPELINE_CLASS`~~：✅ `PlanningPipelineNode` Facade 类已实现
- ~~`TODO_MISSION_CONFIG_DRIVEN_NODE`~~：✅ 单节点 + `mission` 参数驱动后端选择已实现
- ~~`TODO_E2E_INTERFACE_DOC_SYNC`~~：✅ M1-M5 设计文档已同步更新

### 8.4.1 A/B 对比验证记录
- 统一管线通过 `planner:=unified` 启用，旧节点保持不变（默认行为）
- 统一输出话题：`planning/pathlimits`
- 控制层通过 `planner` arg 自动映射到正确话题
- 验证命令：
  ```bash
  # 旧节点（默认）
  roslaunch fsd_launch trackdrive.launch simulation:=true bag:=...
  # 统一节点
  roslaunch fsd_launch trackdrive.launch simulation:=true bag:=... planner:=unified
  ```
- 对比指标：`rostopic hz planning/pathlimits`、Curvature exceeds 告警数、GENERAL FAILSAFE 触发数、控制输出连续性

### 8.5 颜色语义修正（M0，当前延期但必须补齐）
- 目标：颜色规则改为 HUAT 语义（LEFT=RED / RIGHT=BLUE）并打通消息枚举映射。
- 说明：本项当前延期，不阻断架构收敛，但属于后续必须补齐的上限阻塞项。
- 影响：复杂工况（错锥/堆叠/遮挡）下边界分配误差会更高，影响圈速上限与稳定性。
- 待完成：
  - `TODO_COLOR_ENUM_MAPPING`：HUAT_Cone 消息枚举 LEFT=RED / RIGHT=BLUE
  - `TODO_COLOR_PERCEPTION_INTEGRATION`：感知侧颜色标注输出
  - `TODO_COLOR_PLANNING_BOUNDARY`：规划侧按颜色分配左右边界
