# Topic/Frame/Param 统一契约草案（V1）

> 状态：Draft（2026-02-13）  
> 目的：在不改算法的前提下，先统一主链路命名与边界，降低隐式耦合和 remap 依赖。

---

## 1. TopicContract（V1）

### 1.1 Canonical Topics（单一真值）

| 链路 | Canonical Topic | Producer | Consumer |
|---|---|---|---|
| 感知输出 | `/perception/lidar_cluster/detections` | `perception_ros` | `localization_ros` |
| 定位车体状态 | `/localization/car_state` | `localization_ros` | `planning_ros`, `control_ros` |
| 定位锥桶地图 | `/localization/cone_map` | `localization_ros` | `planning_ros`, `fsd_visualization` |
| 规划路径 | `/planning/pathlimits` | `planning_ros` | `control_ros`, `fsd_visualization` |
| 八字接近终点 | `/planning/skidpad/approaching_goal` | `planning_ros` | `control_ros` |
| 控制指令 | `/vehicle/cmd` | `control_ros` | `vehicle_interface_ros`, `simulation_ros` |
| 统一诊断总线 | `/diagnostics` | 各入口节点 | 运维/监控 |

### 1.2 Legacy Alias（兼容映射）

| Legacy Alias | Canonical Topic | 兼容策略 |
|---|---|---|
| `/vehcileCMDMsg` | `/vehicle/cmd` | 控制层可选双发布；接口层可选双订阅 |
| `/Carstate` | `/localization/car_state` | 控制层可选双订阅，默认订阅 canonical |
| `/coneMap` | `/localization/cone_map` | 仿真/调试链路仅兼容，不作为主链 |
| `/skidpad_detection_node/approaching_goal` | `/planning/skidpad/approaching_goal` | 控制层可选双订阅 |

### 1.3 Topic 规则

1. 主链路组件默认只依赖 canonical topic。  
2. legacy alias 仅用于过渡窗口，必须可关闭并可观测（diagnostics 或日志）。  
3. 不允许在核心业务代码中硬编码绝对 legacy topic。

---

## 2. FrameContract（V1）

| Frame | 语义 | 典型发布方 | 典型消费者 |
|---|---|---|---|
| `world` | 全局 ENU 坐标系 | `localization_ros`（`world->base_link`） | planning/control/viz |
| `base_link` | 车体坐标系 | TF 链路中间节点 | perception/localization/planning |
| `velodyne` | 雷达坐标系 | LiDAR/仿真输入 | perception |
| `imu` | IMU 坐标系 | 车端接口/桥接 | localization |

### 2.1 Frame 规则

1. `HUAT_PathLimits.header.frame_id` 必须为 `world`。  
2. `HUAT_CarState.header.frame_id` 必须为 `world`。  
3. 检测消息 `HUAT_ConeDetections.header.frame_id` 默认为 `velodyne`。  
4. 同一时间戳下，`world->base_link` TF 与 `car_state` 必须同源。

---

## 3. ParamNamespace（V1 目标）

### 3.1 命名空间分层

| 层级 | 命名约定 | 示例 |
|---|---|---|
| 组件私有参数 | `~` 私有命名空间 | `~diagnostics_rate_hz` |
| 组件 topic 参数 | `~topics/*` | `~topics/cmd` |
| 兼容开关参数 | `~compat/*` | `~compat/enable_legacy_topics` |
| 框架/全局参数 | `/` 根命名空间（限制使用） | `/use_sim_time` |

### 3.2 参数规则

1. 业务配置优先使用私有命名空间，不允许“私有读取失败后回退全局”作为默认机制。  
2. 兼容链路参数必须显式命名到 `~compat/*`，避免与主链参数混淆。  
3. Topic 相关参数统一放在 `~topics/*`，减少散落字符串。

---

## 4. 迁移阶段（建议）

1. **阶段 A（当前）**：默认 canonical，保留 legacy alias 开关。  
2. **阶段 B**：将 legacy alias 默认关闭，仅在回归/赛事脚本中按需启用。  
3. **阶段 C**：删除 legacy alias 代码路径，保留迁移文档和回退脚本。

---

## 5. 对应基线快照

- `docs/system_specifications/baseline/2026-02-13/regression_metrics.md`  
- `docs/system_specifications/baseline/2026-02-13/runtime_endpoints_snapshot.txt`  
- `docs/system_specifications/baseline/2026-02-13/legacy_alias_refs_snapshot.txt`
