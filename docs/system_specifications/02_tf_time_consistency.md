# TF/时间/单位一致性检查清单

> 本文档定义了系统运行时的一致性检查项，以及自检输出规范，用于快速定位"看起来像算法问题"的工程问题。

---

## 目录

1. [TF链检查清单](#1-tf链检查清单)
2. [时间同步检查清单](#2-时间同步检查清单)
3. [单位一致性检查清单](#3-单位一致性检查清单)
4. [自检输出定义](#4-自检输出定义)
5. [常见问题排查指南](#5-常见问题排查指南)

---

## 1. TF链检查清单

### 1.1 必需的TF变换

| 变换路径 | 发布者 | 频率 | 检查项 |
|----------|--------|------|--------|
| `world → base_link` | localization_node | 100 Hz | 定位节点运行时必须存在 |
| `base_link → velodyne` | static_transform_publisher | 静态 | launch文件配置 |
| `base_link → imu` | static_transform_publisher | 静态 | launch文件配置（**条件必需**：启用畸变补偿时 MUST 存在，否则可省略但需在 health report 声明） |

### 1.2 TF检查命令

```bash
# 检查TF树完整性
rosrun tf tf_echo world base_link

# 查看TF树结构
rosrun tf view_frames
# 生成 frames.pdf

# 检查特定变换的延迟
rosrun tf tf_monitor world base_link

# 检查TF时间戳是否合理
rostopic echo /tf --noarr | grep -A 5 "frame_id"
```

### 1.3 TF检查项

| 检查项 | 期望值 | 失败表现 | 代码位置 |
|--------|--------|----------|----------|
| world→base_link 存在 | ✓ | TF lookup失败 | `location.cpp:383-391` |
| 变换时间戳非零 | ✓ | 使用过期变换 | 所有TF查询 |
| 变换延迟 | < 50ms | 坐标跳变 | `tf_monitor` |
| 静态变换漂移 | 0 | 定位误差累积 | launch文件 |
| frame_id 命名一致 | ✓ | TF链断裂 | 各节点配置 |
| detections frame_id 合规 | `"velodyne"` | 坐标系不匹配（见 Doc-01 §1.2） | 感知节点 |
| pathlimits frame_id 合规 | `"world"` | 控制层坐标系误用 | 规划节点 |
| base_link→imu 存在（条件） | 畸变补偿启用时 ✓ | 点云畸变补偿失效 | launch文件 |
| TF stamp 与 car_state 一致 | 偏差 ≤1ms | TF 外推/回插 | `location.cpp` |

### 1.4 Frame ID 命名规范

| Frame ID | 用途 | 发布者 | 说明 |
|----------|------|--------|------|
| `world` | 全局参考系 | localization_node | 固定坐标系 |
| `base_link` | 车体坐标系 | localization_node | 车辆中心 |
| `velodyne` | LiDAR坐标系 | 静态变换 | 点云原始坐标系 |
| `imu` | IMU坐标系 | 静态变换 | IMU传感器坐标系 |

**常见错误**:
- `World` vs `world` (大小写不一致)
- `base_link` vs `baselink` (下划线缺失)
- `velodyne` vs `lidar` (命名不统一)

---

## 2. 时间同步检查清单

### 2.1 时间戳来源检查

| 数据源 | 期望时间戳来源 | 检查方法 | 问题表现 |
|--------|----------------|----------|----------|
| LiDAR点云 | 硬件时间戳 | `rostopic echo /velodyne_points | grep stamp` | 时间戳为零或跳变 |
| INS数据 | GPS时间 | `rostopic echo /sensors/ins | grep Time` | 与ROS时间差异大 |
| 锥桶检测 | 继承点云时间戳 | 检测消息header.stamp | 使用了 `ros::Time::now()` |
| 车辆状态 | 继承INS时间戳 | 状态消息header.stamp | 时间戳不一致 |

### 2.2 时间同步检查命令

```bash
# 检查消息时间戳与当前时间的差异
rostopic hz /perception/lidar_cluster/detections
rostopic hz /localization/car_state

# 检查时间戳是否为零
rostopic echo /perception/lidar_cluster/detections | grep -A 2 "header:"

# 检查两个话题的时间戳差异
# 方法：录制bag后分析
rosbag info recorded.bag

# 使用 rqt_plot 可视化时间戳
rqt_plot /topic1/header/stamp /topic2/header/stamp
```

### 2.3 时间同步检查项

| 检查项 | 期望值 | 失败表现 | 影响范围 |
|--------|--------|----------|----------|
| 点云时间戳非零 | ✓ | 使用 `ros::Time::now()` 回退 | 畸变补偿失效 |
| INS时间戳连续 | ✓ | 时间跳变 | 状态插值失败 |
| 检测-状态时间差 | < 100ms | 数据关联错误 | 规划层同步失败 |
| pathlimits.header.stamp 非零 | ✓ | 延迟链断裂 | 评估统计失真 |
| pathlimits.stamp 非零 | ✓ | 无法评估规划 compute_time | 性能分析失真 |
| pathlimits.stamp ≥ header.stamp | ✓ | 时间语义反转 | 指标异常 |
| 仿真时间一致性 | ✓ | `/use_sim_time` 配置错误 | 回放时间错乱 |

### 2.4 时间同步机制

#### ApproximateTime 同步（规划层）

**代码位置**: `skidpad_detection_node.cpp:19-25`

```cpp
using SyncPolicy = message_filters::sync_policies::ApproximateTime<ConeMsg, StateMsg>;
sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), cone_sub_, car_state_sub_);
```

**检查项**:
- 队列大小是否足够（默认10）
- 时间戳差异是否在容忍范围内

#### INS状态插值（定位层）

**代码位置**: `location.cpp:753-846`

```cpp
// 环形缓冲区存储最近200条INS状态
static constexpr size_t kInsBufferSize = 200;
std::deque<StampedIns> ins_buffer_;
```

**检查项**:
- 缓冲区是否有足够数据
- 目标时间戳是否在缓冲区范围内

### 2.5 仿真时间配置

```xml
<!-- launch文件中必须配置 -->
<param name="/use_sim_time" value="$(arg simulation)"/>
```

**检查命令**:
```bash
# 检查仿真时间参数
rosparam get /use_sim_time

# 回放bag时必须设置
rosbag play --clock recorded.bag
```

---

## 3. 单位一致性检查清单

### 3.1 角度单位

| 参数 | 单位 | 位置 | 检查项 |
|------|------|------|--------|
| 航向角 (theta/yaw) | radians | HUAT_CarState | 逆时针为正 |
| 航向角 (Heading) | degrees | HUAT_InsP2 | 正北0°顺时针为正 |
| 偏航角速度 (Wz) | rad/s | HUAT_CarState | - |
| 俯仰角 (Pitch) | degrees | HUAT_InsP2 | 抬头为正 |
| 横滚角 (Roll) | degrees | HUAT_InsP2 | 右倾为正 |

**常见转换**:
```cpp
// 度 → 弧度
double rad = deg * M_PI / 180.0;

// 弧度 → 度
double deg = rad * 180.0 / M_PI;

// INS航向(度,顺时针) → 数学角度(弧度,逆时针)
// 注意：以下公式仅适用于 ENU 坐标系（X东Y北）
// double math_yaw = -(heading - 90.0) * M_PI / 180.0;

// 本系统采用 Local-Start 坐标系（见 Doc-01 §4.1），正确转换为：
// theta = -(Heading - Heading_init) * M_PI / 180.0
// 其中 Heading_init 为系统启动时的 INS Heading 值（yaw0 偏置）
```

> **适用前提**: 上述 Local-Start 转换要求系统启动时记录 `Heading_init`。若当前实现使用 ENU 公式，需确认 `world` 坐标系定义是否与 Doc-01 §4.1 一致。

### 3.2 速度单位

| 参数 | 单位 | 位置 | 说明 |
|------|------|------|------|
| 纵向速度 V | m/s | HUAT_CarState | 前向为正 |
| 横向速度 Vy | m/s | HUAT_CarState | 左向为正 |
| 北向速度 Vn | m/s | HUAT_InsP2 | NED坐标系 |
| 东向速度 Ve | m/s | HUAT_InsP2 | NED坐标系 |
| 目标速度 | m/s | HUAT_PathLimits | 规划输出 |

**速度上限参考**:
| 模式 | 速度上限 | 值 |
|------|----------|-----|
| 安全模式 | speed_cap_safe | 11.11 m/s (40 km/h) |
| 快速模式 | speed_cap_fast | 13.33 m/s (48 km/h) |

### 3.3 距离单位

| 参数 | 单位 | 位置 | 说明 |
|------|------|------|------|
| 位置坐标 (x, y) | meters | 所有消息 | - |
| 锥桶距离 obj_dist | meters | HUAT_ConeDetections | - |
| 曲率 | 1/m | HUAT_PathLimits | 曲率半径的倒数 |
| LiDAR到IMU距离 | meters | location_common.yaml | 1.87 m |
| 差分龄期 Age | 0.1秒 | HUAT_InsP2 | 值30 = 3.0秒（见 Doc-01 §2.3） |
| 地图置信度 confidence_scaled | uint32 | HUAT_Cone.confidence | 量化区间 [0,1000]，等价于 score×1000 |
| 检测置信度 confidence_score | float | HUAT_ConeDetections.confidence[] | 连续区间 [0,1] |

### 3.4 加速度单位

| 参数 | 单位 | 位置 | 说明 |
|------|------|------|------|
| 纵向加速度 Ax | m/s² | HUAT_CarState | 前向为正 |
| 横向加速度 Ay | m/s² | HUAT_CarState | 左向为正 |
| IMU加速度 | m/s² | HUAT_InsP2 | - |

### 3.5 控制量单位

| 参数 | 单位 | 范围 | 说明 |
|------|------|------|------|
| 油门 throttle | 归一化 | [-1, 1] | 正值加速，负值制动 |
| 转向 steering_angle | 归一化 | [-1, 1] | 左负右正 |
| 底层转向 steering | uint8 | 0-255 | 执行器原始值 |
| 底层制动 brake_force | uint8 | 0-255 | 执行器原始值 |

---

## 4. 自检输出定义

### 4.1 话题频率统计

**建议实现**: 独立监控节点

```yaml
# 输出格式示例
topic_health:
  - topic: "/perception/lidar_cluster/detections"
    expected_hz: 10
    actual_hz: 9.8
    status: "OK"
    frame_id_ok: true          # frame_id 是否为 "velodyne"（Doc-01 §1.2）
    invalid_color_type_count: 0 # color_types 非法枚举值计数
    avg_confidence_score: 0.65  # 平均 confidence_score
    empty_frame_count: 0        # points[] 为空的帧数
  - topic: "/localization/car_state"
    expected_hz: 100
    actual_hz: 98.5
    status: "OK"
  - topic: "/localization/cone_map"
    expected_hz: 100
    actual_hz: 97.2
    status: "OK"
    confidence_scaled_distribution: {min: 180, max: 940, mean: 612.0} # 量化置信度分布 [0,1000]
    id_churn_per_sec: 0.5       # 单位时间新增 id 数
  - topic: "/planning/pathlimits"
    expected_hz: 20
    actual_hz: 15.2
    status: "WARN"
    empty_path_rate: 0.02       # 空路径帧占比
    curvature_violation_count: 0 # |curvature| > 0.222 的违规点数
    stamp_nonzero_ratio: 1.0     # stamp 非零占比
    stamp_order_ok_ratio: 1.0    # stamp >= header.stamp 占比
  - topic: "/control/command"
    expected_hz: 100
    actual_hz: 99.2
    status: "OK"
    cmd_age_p95_ms: 18.0         # 95分位 age，目标 < 100ms（Doc-01 §3.5）
    estop_cmd_count: 0           # racing_status=E_STOP 的命令次数
```

### 4.2 时间差分布统计

**建议输出**:

| 统计量 | 说明 | 阈值 |
|--------|------|------|
| mean_diff | 平均时间差 | < 50ms |
| max_diff | 最大时间差 | < 100ms |
| std_diff | 时间差标准差 | < 20ms |
| out_of_sync_count | 超阈值次数 | < 5/min |

### 4.3 TF Lookup 统计

**建议输出**:

```yaml
tf_health:
  transform: "world_to_base_link"
  success_rate: 0.998
  avg_latency_ms: 2.3
  max_latency_ms: 15.6
  timeout_count: 2
  extrapolation_count: 0
  imu_tf_present: true          # base_link→imu 是否存在
  distortion_compensation: true  # 畸变补偿是否启用（与 imu_tf_present 联动）
  tf_car_state_diff_ms: 0.3     # TF stamp 与 car_state.header.stamp 的最大偏差
```

### 4.4 延迟统计

**延迟口径定义（MUST）**:

| 口径 | 定义 | 计算方式 | 用途 |
|------|------|----------|------|
| **age** | 消息从产生到被消费的时间 | `now() - msg.header.stamp` | 判断数据新鲜度 |
| **compute_time** | 模块内部处理耗时 | `stamp_out - stamp_in` | 判断算法性能 |

**建议输出**:

| 链路段 | 延迟定义 | 口径 | 计算字段 | 阈值 |
|--------|----------|------|----------|------|
| 感知延迟 | 点云采集 → 锥桶检测输出 | compute_time | `detections.header.stamp - velodyne_points.header.stamp` | < 100ms |
| 定位延迟 | INS数据 → 状态输出 | compute_time | `car_state.header.stamp - ins.header.stamp` | < 50ms |
| 规划延迟 | 检测输入 → 路径输出 | compute_time | `pathlimits.stamp - pathlimits.header.stamp`（见 Doc-01 §3.1） | < 50ms |
| 控制延迟 | 路径输入 → 命令输出 | compute_time | `command.header.stamp - pathlimits.header.stamp` | < 10ms |
| 端到端延迟 | 点云采集 → 控制命令 | age | `command.header.stamp - detections.header.stamp`（需时间戳传递链完整） | < 200ms |

> **注意**: 分段 compute_time 阈值之和（100+50+50+10=210ms）超过端到端 age 阈值（200ms），因为各段存在流水线并行。端到端延迟以 age 口径为准。

### 4.5 自检节点设计建议

```python
# 伪代码示例
class HealthMonitor:
    def __init__(self):
        self.topic_stats = {}
        self.tf_stats = {}
        self.latency_stats = {}
    
    def check_topic_frequency(self, topic, msg):
        # 统计消息频率
        pass
    
    def check_timestamp_consistency(self, topic, msg):
        # 检查时间戳是否合理
        pass
    
    def check_tf_availability(self):
        # 检查TF变换是否可用
        pass
    
    def publish_health_report(self):
        # 发布健康报告
        pass
```

### 4.6 A3 文件侧通道兼容性自检

> 目标：验证 `$HOME/autoStartGkj/command` 已降级为“运维兼容”，不再构成主链路依赖（见 Doc-01 §3.6）。

**当前实现**:
- 控制节点已发布 `/control/diagnostics`，并聚合到统一 `/diagnostics`，包含 `mode_source`、`file_mode_fallback_used`、`external_stop_source`、文件读错计数等字段。
- 定位节点已发布 `/localization/diagnostics`，并聚合到统一 `/diagnostics`，新增 `localization_entry_health`（`source/backend/has_carstate` 等字段）。
- 规划节点已发布 `/planning/diagnostics`，并聚合到统一 `/diagnostics`，新增 `planning_entry_health`（`mission/backend/local_tf_valid/lap_mode` 等字段）。
- `diagnostic_aggregator` 已分组到 `/diagnostics_agg`：
  - `ControlEntry/control_entry_health`
  - `LocalizationEntry/localization_entry_health`
  - `PlanningEntry/planning_entry_health`

**建议输出**:

```yaml
control_entry_health:
  mode_source: "param"                 # param | file
  file_mode_fallback_used: false       # 本次运行是否触发文件回退
  file_mode_read_error_count: 0        # 文件读取失败次数
  external_stop_source: "disabled"     # disabled | file | ros
  external_stop_file_open_error_count: 0
localization_entry_health:
  source: "imu"                        # startup | imu | carstate | cone | cone_waiting_carstate
  backend: "mapper"                    # mapper | factor_graph
  has_carstate: true
planning_entry_health:
  mission: "high_speed"
  backend: "high_speed_tracking"       # high_speed_tracking | line_detection | skidpad_detection
  local_tf_valid: true
  lap_mode: "MAP_BUILD_SAFE"           # MAP_BUILD_SAFE | FAST_LAP | n/a
```

**判定规则（MUST）**:
- 标准赛项与评估回放中，`mode_source` **MUST** 为 `param`。
- 标准赛项与评估回放中，`file_mode_fallback_used` **MUST** 为 `false`。
- 仿真回放（`/use_sim_time=true`）中，`external_stop_source` **MUST** 为 `disabled`，且文件开关错误不应影响 `/control/command` 连续发布。

**建议采集信号**:
- `/control/command`：验证命令连续性与时效。
- `/diagnostics`：读取 `control_entry_health` / `localization_entry_health` / `planning_entry_health`（统一诊断总线聚合结果）。
- `/diagnostics_agg`：验证分组路径 `ControlEntry/*`、`LocalizationEntry/*`、`PlanningEntry/*`（分栏显示）。
- `/rosout` 或 `/rosout_agg`：统计“文件打开失败”类日志频次（用于兼容链路观测）。
- launch 参数快照：`control_mode` 是否显式传入（见 `fsd_launch/launch/subsystems/control.launch`）。

---

## 5. 常见问题排查指南

### 5.1 TF链断裂

**症状**:
- `Transform from X to Y failed`
- 可视化节点无数据
- 坐标变换返回NaN

**排查步骤**:
1. 检查 `rosrun tf view_frames` 输出
2. 确认定位节点是否运行
3. 检查 frame_id 命名是否一致
4. 检查静态变换是否发布

### 5.2 时间戳为零

**症状**:
- 数据关联失败
- 插值返回边界值
- ApproximateTime同步失败

**排查步骤**:
1. 检查传感器驱动配置
2. 确认是否使用硬件时间戳
3. 检查代码中是否有 `ros::Time::now()` 替换

### 5.3 时间不同步

**症状**:
- 规划结果跳变
- 控制响应延迟
- 数据关联错误

**排查步骤**:
1. 检查 `/use_sim_time` 配置
2. 确认所有节点使用相同时间源
3. 检查 ApproximateTime 队列大小
4. 分析 bag 文件中的时间戳分布

### 5.4 单位不一致

**症状**:
- 角度计算错误
- 速度规划异常
- 控制输出震荡

**排查步骤**:
1. 检查 INS 数据的角度单位
2. 确认控制器的角度输入单位
3. 检查速度单位是否为 m/s
4. 验证曲率计算公式
5. 检查 `car_state.theta` 连续性：相邻帧 Δtheta 不应超过 `Wz × dt × 2`（允许 2 倍余量）
6. 统计 `theta` 跳变次数（`yaw_jump_count`）：单帧 Δtheta > 0.1 rad 视为跳变

### 5.5 仿真时间问题

**症状**:
- bag 回放时节点无响应
- 时间戳与系统时间不匹配
- TF 查询超时

**排查步骤**:
1. 确认 `/use_sim_time: true`
2. 使用 `rosbag play --clock`
3. 检查节点是否正确处理仿真时间

---

## 附录：检查脚本示例

```bash
#!/bin/bash
# system_health_check.sh

echo "=== TF Check ==="
rosrun tf tf_echo world base_link 2>&1 | head -5

echo -e "\n=== Topic Frequency ==="
rostopic hz /perception/lidar_cluster/detections &
PID=$!
sleep 5
kill $PID 2>/dev/null

echo -e "\n=== Timestamp Check ==="
rostopic echo -n 1 /localization/car_state | grep -A 3 "header:"

echo -e "\n=== Time Sync Check ==="
rosparam get /use_sim_time

echo -e "\n=== Health Report Complete ==="
```

## 附录B：B1/B2/B3 一键验收脚本

```bash
bash scripts/b123_contract_validation.sh /home/kerwin/rosbag/track.bag 2.0
```

**脚本检查项**:
- `/planning/pathlimits`：`header.frame_id=world`、`header.stamp/stamp` 非零、`stamp>=header.stamp`、`path/target_speeds/curvatures` 1:1。
- `/localization/cone_map`：`header.frame_id=world`、`HUAT_Cone.confidence ∈ [0,1000]`。
- `/perception/lidar_cluster/detections`：`header.frame_id=velodyne`、`confidence[] ∈ [0,1]`。
