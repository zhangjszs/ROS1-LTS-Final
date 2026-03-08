# 端到端接口契约

> 本文档定义了 LiDAR→Location→Planning→Control 全链路的接口规范，确保各模块间的数据交换一致、可追溯。

---

## 目录

1. [LiDAR → Location 接口](#1-lidar--location-接口)
2. [Location → Planning 接口](#2-location--planning-接口)
3. [Planning → Control 接口](#3-planning--control-接口)
4. [坐标系参考系约定](#4-坐标系参考系约定)
5. [时间戳规范](#5-时间戳规范)

---

## 1. LiDAR → Location 接口

### 1.1 消息类型

**Topic**: `/perception/lidar_cluster/detections`
**消息类型**: `autodrive_msgs/HUAT_ConeDetections`

> **约定**: 本文档所有 Topic 名称均使用绝对路径（带前导 `/`）。

### 1.2 字段契约表

| 字段名 | 类型 | 单位/范围 | 说明 | 来源 |
|--------|------|-----------|------|------|
| `header.stamp` | `ros::Time` | - | 点云采集时刻的硬件时间戳 | LiDAR驱动 |
| `header.frame_id` | `string` | `"velodyne"` | 点云坐标系 | LiDAR驱动 |
| `header.seq` | `uint32` | - | 消息序列号（ROS自动填充） | ROS |
| `points[]` | `Point32[]` | meters | 锥桶中心坐标 (x, y, z)，坐标系与 `header.frame_id` 一致 | 聚类算法 |
| `maxPoints[]` | `Point32[]` | meters | 锥桶边界最大点，坐标系与 `header.frame_id` 一致 | 聚类算法 |
| `minPoints[]` | `Point32[]` | meters | 锥桶边界最小点，坐标系与 `header.frame_id` 一致 | 聚类算法 |
| `pc[]` | `PointCloud2[]` | - | 每个锥桶的点云簇 | 聚类算法 |
| `pc_whole` | `PointCloud2` | - | 整体点云 | 聚类算法 |
| `confidence[]` | `float32[]` | [0.0, 1.0] | 单帧置信度评分（canonical 名称：`confidence_score`） | ConfidenceScorer |
| `obj_dist[]` | `float32[]` | meters | 锥桶到车辆的距离 | 聚类算法 |
| `color` | `String` | `'b'`/`'y'`/`'r'`/`'n'` | 整体颜色标识（**已废弃**，消费者禁止使用；期望值为空字符串或 `'n'`） | 颜色分类 |
| `color_types[]` | `uint8[]` | 0-5 | 每个锥桶的颜色类型（枚举见 §1.3） | 颜色分类 |

**数组不变量（MUST）**:
- `len(points) == len(confidence) == len(obj_dist) == len(color_types) == len(maxPoints) == len(minPoints) == len(pc)`
- 不满足时该帧消息 **MUST** 被判为 INVALID，下游禁止消费。

**坐标系不变量（MUST）**:
- `points[]`/`maxPoints[]`/`minPoints[]`/`pc[]`/`obj_dist[]` 所在坐标系 **MUST** 与 `header.frame_id` 一致。
- 当前实现中 `header.frame_id="velodyne"`，即数据在 LiDAR 原始坐标系下发布；下游若需 `base_link` 坐标系数据，**MUST** 通过 TF 变换获取。

**pc[] header 继承规则（SHOULD）**:
- `pc[i].header` **SHOULD** 与顶层 `header` 保持一致（相同 `stamp` 和 `frame_id`），以便下游做一致性校验。

### 1.3 颜色编码定义

| 值 | 枚举名 | 颜色 | 说明 |
|----|--------|------|------|
| 0 | `BLUE` | 蓝色 | 左边界锥桶 |
| 1 | `YELLOW` | 黄色 | 右边界锥桶 |
| 2 | `ORANGE_SMALL` | 橙色小 | 起终点标识 |
| 3 | `ORANGE_BIG` | 橙色大 | 起终点标识 |
| 4 | `NONE` | 无色 | 未分类 |
| 5 | `RED` | 红色 | 特殊标识 |

**枚举合法性（MUST）**: `color_types[]` 值域 **MUST** 在 0-5 范围内；未知/无法分类的值 **MUST** 映射为 `NONE(4)`，并在 health report 中计数。

### 1.4 置信度计算规则

置信度由 `ConfidenceScorer` 计算，采用多维度加权评分：

| 维度 | 权重 | 评分依据 |
|------|------|----------|
| 尺寸约束 | 0.30 | 高度[0.15, 0.5]m, 面积[0.01, 0.15]m² |
| 形状约束 | 0.25 | 纵横比≥1.5, 线性度≤0.85 |
| 密度约束 | 0.20 | 距离自适应点数密度 |
| 强度约束 | 0.15 | 反射强度均值≥30 |
| 位置约束 | 0.10 | 地面高度≤0.5m |

**入图阈值**: `min_confidence_to_add: 0.2`（canonical 参数名，评估配置中别名 `confidence_threshold` 指向同一语义）

**置信度口径统一（MUST）**:
- 感知层 `HUAT_ConeDetections.confidence[]` 使用 `confidence_score ∈ [0,1]`。
- 地图层 `HUAT_Cone.confidence` 使用 `confidence_scaled ∈ [0,1000]`，编码规则：`confidence_scaled = round(clamp(confidence_score,0,1) * 1000)`。
- Planning/Control 若按阈值比较，**MUST** 先解码：`confidence_score = confidence_scaled / 1000.0`，禁止直接拿 `uint32` 与 `[0,1]` 阈值比较。
- `confidence_scaled` **MUST NOT** 被解释为“观测次数累计值”。

### 1.5 ID机制

当前版本**不保证锥桶ID的全局唯一性**，每次检测独立输出。地图层负责ID分配与跟踪。

### 1.6 时间戳不变量（MUST）

- `header.stamp` **MUST** 为硬件时间戳且非零；若为零 **MUST** 拒绝发布或标记为 INVALID（禁止 silent fallback 到 `ros::Time::now()`）。
- `header.stamp` **MUST** 单调递增（允许硬件抖动 ≤1ms）。

---

## 2. Location → Planning 接口

### 2.1 车辆状态消息

**Topic**: `/localization/car_state`
**消息类型**: `autodrive_msgs/HUAT_CarState`

| 字段名 | 类型 | 单位 | 说明 | 参考系 |
|--------|------|------|------|--------|
| `header.stamp` | `ros::Time` | - | 状态估计时刻 | - |
| `header.frame_id` | `string` | `"world"` | 全局坐标系 | world |
| `car_state.x` | `float64` | meters | 全局X坐标 | world |
| `car_state.y` | `float64` | meters | 全局Y坐标 | world |
| `car_state.theta` | `float64` | radians | 航向角（逆时针为正，零向与 world X 轴对齐，见 §4.1） | world |
| `car_state_front` | `Point` | meters | 前轴中心位置 | world |
| `car_state_rear` | `Point` | meters | 后轴中心位置 | world |
| `V` | `float32` | m/s | 纵向速度 | base_link |
| `W` | `float32` | rad/s | 偏航角速度（已废弃） | base_link |
| `A` | `float32` | m/s² | 加速度（已废弃） | base_link |
| `Vy` | `float32` | m/s | 横向速度 | base_link |
| `Wz` | `float32` | rad/s | 偏航角速度 | base_link |
| `Ax` | `float32` | m/s² | 纵向加速度 | base_link |
| `Ay` | `float32` | m/s² | 横向加速度 | base_link |

**混合参考系消费规则（MUST）**:
- `header.frame_id` 仅约束位置/航向字段（`car_state.x/y/theta`、`car_state_front`、`car_state_rear`），这些字段在 `world` 坐标系下。
- 速度/加速度/角速度字段（`V`、`Vy`、`Wz`、`Ax`、`Ay`）固定在 `base_link` 坐标系下，消费者 **禁止** 假设与 `header.frame_id` 同一参考系。

### 2.2 锥桶地图消息

**Topic**: `/localization/cone_map`
**消息类型**: `autodrive_msgs/HUAT_ConeMap`

| 字段名 | 类型 | 说明 |
|--------|------|------|
| `header.stamp` | `ros::Time` | 地图更新时刻 |
| `header.frame_id` | `string` | `"world"` |
| `cone[]` | `HUAT_Cone[]` | 地图中的锥桶数组 |

**HUAT_Cone 结构**:

| 字段名 | 类型 | 说明 |
|--------|------|------|
| `position_baseLink` | `Point32` | 车体坐标系位置（由 `T_baselink_world(header.stamp)` 实时计算） |
| `position_global` | `Point32` | 全局坐标系位置（地图固定值，权威字段） |
| `id` | `uint32` | 全局唯一ID |
| `confidence` | `uint32` | 量化置信度（canonical 名称：`confidence_scaled`，范围 [0,1000]，语义等价于 `confidence_score×1000`） |
| `type` | `uint32` | 锥桶类型/颜色，**MUST** 采用与 §1.3 相同的枚举值（0-5） |

**双坐标字段一致性不变量（MUST）**:
- `position_baseLink` 与 `position_global` 的关系：`position_baseLink == T_baselink_world(header.stamp) × position_global`
- 下游消费者 **MUST** 优先使用 `position_global`（权威字段）；`position_baseLink` 为便利字段，仅在已知 TF 可用时使用。
- 当 TF 不可用时，`position_baseLink` **SHOULD** 置为 `(NaN, NaN, NaN)`，并在 health report 中标记。

**地图置信度定义（MUST）**:
- `confidence`（`confidence_scaled`）为地图锥桶置信度的量化表达，合法值域 `[0,1000]`。
- 与检测层 `confidence_score` 为同一语义但不同编码；跨模块传递时 **MUST** 按“score↔scaled”规则转换。
- 任何 `confidence > 1000` 或 `< 0` 的样本 **MUST** 计为契约违规（health report 计数）。

**ID 稳定性规则（MUST）**:
- **真实锥桶（由检测直接入图）**：`id` 在单次会话（从启动到关闭）内全局唯一且稳定，同一物理锥桶的 `id` 不变。
- **真实锥桶**：`id` 不可复用（已删除锥桶的 `id` 不再分配给新锥桶）。
- 系统重启后 `id` 重置。

**虚拟锥（插值锥）ID 规则（MUST）**:
- 缺锥补偿生成的虚拟锥 `id` **MUST** 为非零值，禁止使用 `id=0`。
- 虚拟锥 `id` **MUST** 在会话内全局唯一（与真实锥桶共享同一分配器）。
- 虚拟锥 `id` **MAY** 跨帧变化；下游消费者 **MUST NOT** 将其用于跨帧数据关联。
- 虚拟锥语义标识保持：`type=NONE(4)`，`confidence` 使用 `min_confidence_for_interpolation` 按 `×1000` 量化后编码。

### 2.3 定位质量指标

当前版本**未显式输出定位质量/协方差**，但可通过以下信号间接判断：

| 信号 | 来源 | 说明 |
|------|------|------|
| INS状态 | `HUAT_InsP2.Status` | 0=未初始化, 1=姿态初始化, 2=组合导航正常 |
| 卫星数 | `HUAT_InsP2.NSV1` | ≥8 为良好 |
| 差分龄期 | `HUAT_InsP2.Age` | 单位：0.1秒；≤30 即 ≤3.0秒 为良好 |
| 异常状态机 | `AnomalyStateMachine` | TRACKING/DEGRADED/LOST/RELOC |

**定位质量消费规则（SHOULD）**:
- 当 `Status < 2` 或 `Age > 30`（即 >3.0s）时，Planning **SHOULD** 进入降级模式（限速/禁用高速功能）。
- 【未来实现项】：建议将定位质量信号通过 `/diagnostics` topic 或独立 topic 对外发布，以便 Planning 直接消费。

### 2.4 TF发布

定位节点发布 `world → base_link` 变换：

```
parent_frame: "world"
child_frame: "base_link"
translation: (x, y, z=0)
rotation: (qx, qy, qz, qw) from yaw
```

**TF 与 car_state 时间一致性（MUST）**:
- `world → base_link` TF 的 `stamp` **MUST** 与同一时刻 `car_state.header.stamp` 同源（来自同一状态估计输出）。
- 允许偏差阈值：≤1ms。
- 若 TF 与 car_state 时间戳不一致，下游 TF 查询可能外推/回插，导致定位跳变。

---

## 3. Planning → Control 接口

### 3.1 路径限制消息

**Topic**: `/planning/pathlimits`
**消息类型**: `autodrive_msgs/HUAT_PathLimits`

**兼容话题（Deprecated）**:
- `/planning/high_speed_tracking/pathlimits/partial`
- `/planning/high_speed_tracking/pathlimits/full`
- 上述话题仅用于历史可视化兼容，**MUST NOT** 作为 Control 主输入。
- 可视化默认订阅 **MUST** 使用 `/planning/pathlimits`；仅当显式开启兼容参数（`compat/enable_legacy_partial_full=true`）时才允许订阅 `partial/full`。

| 字段名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| `header.stamp` | `ros::Time` | - | 规划所用输入数据的时间戳（即最新输入 detections/car_state 的 header.stamp） |
| `header.frame_id` | `string` | `"world"` | 全局坐标系 |
| `stamp` | `time` | - | 规划计算完成时刻（`ros::Time::now()`），用于计算规划处理耗时 |
| `path[]` | `Point[]` | meters | 路径点数组（中心线） |
| `tracklimits.left[]` | `HUAT_Cone[]` | - | 左边界锥桶（复用 §2.2 HUAT_Cone 结构，双坐标不变量同样适用） |
| `tracklimits.right[]` | `HUAT_Cone[]` | - | 右边界锥桶 |
| `replan` | `bool` | - | 是否需要重新规划 |
| `target_speeds[]` | `float64[]` | m/s | 每个点的目标速度，与path[] 1:1对应 |
| `curvatures[]` | `float64[]` | 1/m | 每个点的曲率，与path[] 1:1对应 |

**时间戳语义与优先级（MUST）**:
- `header.stamp`：输入数据时间戳（用于延迟链计算：`控制延迟 = t_cmd - header.stamp`）。
- `stamp`：规划输出/计算完成时间（用于计算规划处理耗时：`规划耗时 = stamp - header.stamp`）。
- `line/skidpad/high_speed` 三类规划输出 **MUST** 使用完全一致的语义：`header.stamp`=输入时刻，`stamp`=输出完成时刻。
- 当 `stamp` 未填充（为零）时，延迟计算 **MUST** 回退到仅使用 `header.stamp`。
- 【未来实现项】：若当前代码未填充 `stamp`，需实现。

**Frame 契约（MUST）**:
- `/planning/pathlimits.header.frame_id` **MUST** 为 `world`。
- 当上游输入 frame 为 `velodyne`（检测层）时，规划模块 **MUST NOT** 重写输入语义；仅在输出层统一到 `world`。

**数组长度不变量（MUST）**:
- `len(path) == len(target_speeds) == len(curvatures)`
- 当任意数组为空或长度不等时，该帧 **MUST** 标记为"规划失败"，Control **SHOULD** 保持上一次有效路径（`hold_last_valid_max_frames`）。

**曲率上限不变量（MUST）**:
- `|curvatures[i]|` **MUST** ≤ `curvature_limit: 0.222 1/m`（对应最小转弯半径 ~4.5m）。
- 违规时 **MUST** 置 `replan=true` 或对违规段进行裁剪/降级。

### 3.2 轨迹表达形式

**离散点序列**：
- 路径由一系列 (x, y) 点组成
- 点间距 **MUST** 满足：95% 的点间距在 [0.5, 1.0] 米范围内，最大不超过 2.0 米
- 每个点附带目标速度和曲率信息
- 无时间戳序列，由控制层根据速度推算：`dt_i = ds_i / max(v_i, v_min)`，其中 `ds_i` 为点间距，`v_i` 为该点目标速度，`v_min = 0.5 m/s`（防止零速除零）

### 3.3 控制命令消息

**Topic**: `/control/command`
**消息类型**: `autodrive_msgs/HUAT_ControlCommand`

| 字段名 | 类型 | 范围 | 说明 |
|--------|------|------|------|
| `header.stamp` | `ros::Time` | - | 命令发出时刻（`ros::Time::now()`） |
| `throttle` | `Float32` | [-1, 1] | 油门值，正值加速，负值制动 |
| `steering_angle` | `Float32` | [-1, 1] | 转向角，左负右正 |
| `racing_status` | `uint8` | 见下表 | 比赛状态标志 |

**racing_status 枚举（SHOULD 补齐）**:

| 值 | 状态 | 说明 |
|----|------|------|
| 0 | `IDLE` | 空闲/未启动 |
| 1 | `RUNNING` | 正常行驶 |
| 2 | `FINISH` | 完赛 |
| 3 | `E_STOP` | 紧急停车 |

> 【未来实现项】：若当前代码未使用上述枚举，需实现并与底层 `vehicle/cmd` 的 `racing_status` 对齐。

### 3.4 底层车辆命令

**Topic**: `/vehicle/cmd`
**消息类型**: `autodrive_msgs/HUAT_VehicleCmd`

| 字段名 | 类型 | 说明 |
|--------|------|------|
| `steering` | `uint8` | 转向执行值 |
| `brake_force` | `uint8` | 制动力 |
| `pedal_ratio` | `uint8` | 油门踏板比例 |
| `gear_position` | `uint8` | 档位 |
| `working_mode` | `uint8` | 工作模式 |
| `racing_status` | `uint8` | 比赛状态 |
| `checksum` | `uint16` | 校验和 |

### 3.5 控制周期假设

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 控制频率 | 100 Hz | 控制节点运行频率 |
| 规划频率 | 20 Hz | 规划节点运行频率 |
| 感知频率 | 10 Hz | LiDAR点云处理频率 |
| 控制命令 age 上限 | 100 ms | 控制命令的 age（`now() - header.stamp`）不得超过此值；超过时视为过期 |

> **延迟口径说明**: "100ms" 指控制命令的 message age（当前时间与消息 header.stamp 的差），而非控制节点内部处理耗时。端到端延迟预算见 Doc-02 §4.4。

### 3.6 控制入口优先级与文件侧通道策略（A3）

**现状证据**:
- 控制模式参数优先：`control_node` 启动时优先读取私有参数 `~mode`（`src/control_ros/src/control_node.cpp:411-415`）。
- 文件回退存在：仅在 `enable_file_mode_fallback=true` 时回退读取 `$HOME/autoStartGkj/command`（`src/control_ros/src/control_node.cpp:416-427`）。
- 文件停车检查存在：仅在 `enable_external_stop_file=true` 且非仿真模式下读取文件触发停车（`src/control_ros/src/control_node.cpp:103-109`、`src/control_ros/src/control_node.cpp:261-272`）。
- 文件写入节点默认输出同一路径（`src/vehicle_racing_num_ros/launch/vehicle_racing_num.launch:3`、`src/vehicle_racing_num_core/src/racing_num_writer.cpp:9-17`）。

| 通道 | 载体 | 用途 | 契约级别 |
|------|------|------|----------|
| 主链路 | ROS 参数 `~mode`（由 launch `control_mode` 传入） | 控制模式选择 | **MUST** |
| 主链路 | ROS Topic `/planning/pathlimits` + `/control/command` | 闭环控制输入/输出 | **MUST** |
| 兼容链路 | 文件 `$HOME/autoStartGkj/command` | 运维/历史兼容（模式回退、外部停车） | **SHOULD（非主链路）** |

**A3 契约条款**:
- 标准赛项（`trackdrive`/`autocross`/`acceleration`/`skidpad`/`ebs_test`）启动链路 **MUST** 显式提供 `control_mode`，不得把文件通道作为启动前提。
- 文件通道 **MUST NOT** 作为主控制链路依赖；其缺失 **MUST NOT** 导致主链路（参数 + topic）不可用。
- 评估回放与 CI 验证场景中，控制模式来源 **MUST** 为参数链路，不允许依赖文件回退。
- 仿真回放（`/use_sim_time=true`）时，外部停车文件检查 **MUST** 视为关闭（当前实现已在仿真分支跳过）。
- 已实现显式兼容开关参数：`enable_file_mode_fallback`、`enable_external_stop_file`（默认 false）。
- 已实现诊断字段输出：`/control/diagnostics` 与统一 `/diagnostics` 均发布 `mode_source` / `file_mode_fallback_used` / `external_stop_source` 等入口健康字段。
- 已实现 `diagnostic_aggregator` 分组规则：
  - `control_entry_health` → `/diagnostics_agg` 的 `ControlEntry` 分栏；
  - `localization_entry_health`（含 `localization/factor_graph`）→ `LocalizationEntry` 分栏；
  - `planning_entry_health`（含 `planning/high_speed`）→ `PlanningEntry` 分栏。

---

## 4. 坐标系参考系约定

### 4.1 坐标系定义

| 坐标系 | 名称 | 原点 | 方向 |
|--------|------|------|------|
| `world` | 全局坐标系 | 起点位置 | **Local-Start**: X 沿起跑时车头方向、Y 左、Z 上，固定不动 |
| `base_link` | 车体坐标系 | 车辆中心 | X前Y左Z上，随车运动 |
| `velodyne` | LiDAR坐标系 | LiDAR中心 | X前Y左Z上 |
| `imu` | IMU坐标系 | IMU中心 | X前Y左Z上 |

**world 坐标系选择说明（MUST）**:
- 本系统采用 **Local-Start** 约定：`world` 的 X 轴沿系统启动时车头方向，Y 轴向左，Z 轴向上。
- `world` **不是** ENU（东北天）坐标系。INS 输出的 Heading（正北 0°、顺时针）需要经过偏置转换才能对齐到 `world`。
- `car_state.theta` 的零向 **MUST** 与 `world` X 轴对齐，正方向为逆时针（CCW）。
- INS Heading → `car_state.theta` 的转换公式：`theta = -(Heading - Heading_init) × π/180`，其中 `Heading_init` 为系统启动时的 INS Heading 值。
- 【未来实现项】：若当前实现未记录 `Heading_init`（即 `yaw0` 偏置），需在配置中显式记录。

### 4.2 TF树结构

```
world
  └── base_link
        └── velodyne (静态变换)
        └── imu (静态变换，条件必需)
```

**IMU TF 条件要求（MUST）**:
- 当启用点云畸变补偿（IMU 预积分）时，`base_link → imu` **MUST** 存在且为静态变换。
- 当未启用畸变补偿时，`base_link → imu` 可省略，但 **MUST** 在配置/health report 中声明"畸变补偿关闭"。

### 4.3 静态变换参数

| 变换 | 参数 | 值 |
|------|------|-----|
| base_link → velodyne | X偏移 | 配置文件 |
| base_link → velodyne | Y偏移 | 配置文件 |
| base_link → velodyne | Z偏移 | 配置文件 |
| LiDAR到IMU距离 | lidarToIMUDist | 1.87 m |

---

## 5. 时间戳规范

### 5.1 时间戳来源优先级

| 优先级 | 来源 | 使用场景 |
|--------|------|----------|
| 1 | 硬件时间戳 | 传感器数据（点云、INS） |
| 2 | GPS时间 | INS组合导航数据 |
| 3 | `ros::Time::now()` | 回退方案，仅当硬件时间戳为零时 |

### 5.2 时间戳处理规则

1. **点云数据**: 必须使用硬件时间戳，禁止使用 `ros::Time::now()`；若硬件时间戳为零 **MUST** 拒绝发布
2. **INS数据**: 使用GPS周内秒转换为ROS时间
3. **规划输出**: `header.stamp` 使用最新输入数据的时间戳；`stamp` 使用 `ros::Time::now()` 记录计算完成时刻（见 §3.1 时间戳语义）
4. **控制输出**: 使用 `ros::Time::now()`

### 5.3 时间同步机制

| 模块 | 同步方式 | 说明 |
|------|----------|------|
| 规划层 | `ApproximateTime` | 锥桶检测与车辆状态同步 |
| 定位层 | 环形缓冲区插值 | INS状态与检测时间戳对齐 |
| 感知层 | IMU预积分 | 点云畸变补偿 |

---

## 附录：消息定义文件路径

| 消息 | 文件路径 |
|------|----------|
| HUAT_ConeDetections | `src/autodrive_msgs/msg/HUAT_ConeDetections.msg` |
| HUAT_CarState | `src/autodrive_msgs/msg/HUAT_CarState.msg` |
| HUAT_ConeMap | `src/autodrive_msgs/msg/HUAT_ConeMap.msg` |
| HUAT_PathLimits | `src/autodrive_msgs/msg/HUAT_PathLimits.msg` |
| HUAT_ControlCommand | `src/autodrive_msgs/msg/HUAT_ControlCommand.msg` |
| HUAT_VehicleCmd | `src/autodrive_msgs/msg/HUAT_VehicleCmd.msg` |
| HUAT_InsP2 | `src/autodrive_msgs/msg/HUAT_InsP2.msg` |

---

## 附录B：关键参数字典

| Canonical 名称 | 别名 | 单位 | 默认值 | 作用点 | 说明 |
|----------------|------|------|--------|--------|------|
| `min_confidence_to_add` | `confidence_threshold` | - | 0.2 | 检测→入图 | 检测置信度低于此值的锥桶不入图 |
| `curvature_limit` | - | 1/m | 0.222 | 规划输出 | 路径曲率绝对值上限 |
| `speed_cap_safe` | - | m/s | 11.11 | 规划输出 | 安全模式速度上限 |
| `speed_cap_fast` | - | m/s | 13.33 | 规划输出 | 快速模式速度上限 |
| `match_ratio_lost` | - | - | 0.3 | 定位层 | 匹配比率低于此值进入 LOST |
| `chi2_degrade` | - | - | 15.0 | 定位层 | chi² 超过此值进入 DEGRADED |
| `hold_last_valid_max_frames` | - | 帧 | - | 规划→控制 | 规划失败时保持上一有效路径的最大帧数 |
| `control_mode` | `~mode` | - | 赛项指定 | 控制层入口 | 控制模式主链路输入（MUST 显式传入） |
| `enable_file_mode_fallback` | - | bool | false | 控制层入口 | 文件模式回退开关（兼容链路） |
| `enable_external_stop_file` | - | bool | false | 控制层入口 | 文件外部停车开关（兼容链路） |

**速度-曲率一致性（SHOULD）**:
- 规划输出的 `target_speeds[]` **SHOULD** 不超过当前模式对应的速度上限（`speed_cap_safe` 或 `speed_cap_fast`）。
- 高曲率段的目标速度 **SHOULD** 满足横向加速度约束：`v² × |κ| ≤ a_lat_max`。
