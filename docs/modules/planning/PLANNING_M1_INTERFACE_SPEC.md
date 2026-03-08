# PLANNING_M1_INTERFACE_SPEC

## 1. 目标

定义规划到控制的统一接口，满足：

1. 保持现有 `HUAT_PathLimits` 兼容。
2. 增加速度/曲率/时标完整输出。
3. 支持第一圈稳态与后续圈提速模式切换。

本文件为计划阶段文档，不涉及代码改动。

## 2. 现状证据

1. 现有路径消息字段：`path[]`、`tracklimits`、`replan`、`target_speeds[]`、`curvatures[]`。见 `src/autodrive_msgs/msg/HUAT_PathLimits.msg:1`。
2. 现有边界消息字段：`left[]`、`right[]`、`replan`。见 `src/autodrive_msgs/msg/HUAT_TrackLimits.msg:1`。
3. high-speed 默认发布统一消息：`planning/pathlimits`（`output_pathlimits_topic` 默认值）。见 `src/planning_ros/config/high_speed_tracking.yml:4`。
4. `partial/full` 仅保留为历史可视化兼容链路，不作为主控制输入。见 `docs/system_specifications/01_interface_contract.md:201`。
5. high-speed 发布逻辑在主节点里完成。见 `src/planning_ros/src/high_speed_tracking/main.cpp:305`。
6. 直线与八字当前输出仍是 `nav_msgs/Path`。见 `src/planning_ros/src/line_detection_node.cpp:23`、`src/planning_ros/src/skidpad_detection_node.cpp:23`。

## 3. 兼容策略

1. 不直接修改 `HUAT_PathLimits`，避免 ROS message MD5 变化带来的联调风险。
2. 新增 `HUAT_PathLimitsV2`，并保持一段时间双发布。
3. 控制侧先并行订阅 V1/V2，再切主到 V2。

## 4. 建议消息定义（V2）

```text
std_msgs/Header header
time stamp
uint8 profile_mode            # 0=SAFE_LAP,1=FAST_LAP,2=ACCEL,3=SKIDPAD

geometry_msgs/Point[] path    # world frame
float64[] s                   # arc length [m]
float64[] yaw                 # heading [rad]
float64[] curvatures          # [1/m]
float64[] target_speeds       # [m/s]
float64[] target_accels       # [m/s^2]
float64[] target_time         # relative time [s], start from 0

autodrive_msgs/HUAT_TrackLimits tracklimits
bool replan
```

## 5. 接口契约

1. 坐标系：
   1. `path` 必须在 `world`。
   2. 锥桶输入允许 `base_link/velodyne`，内部统一后再导出。
2. 单位：`m, s, rad, m/s, m/s^2, 1/m`。
3. 长度一致性：
   1. `N = len(path)`
   2. `len(s)=len(yaw)=len(curvatures)=len(target_speeds)=len(target_accels)=len(target_time)=N`
4. 时间合法性：
   1. `target_time[0] = 0`
   2. `target_time[i+1] > target_time[i]`
5. 数值合法性：
   1. 所有数组元素必须有限（非 NaN/Inf）
   2. `target_speeds[i] >= 0`
   3. `abs(curvatures[i]) <= kappa_limit_mission`

## 6. 模式语义

1. `SAFE_LAP`：高必第一圈/建图稳态。
2. `FAST_LAP`：高必闭环后提速。
3. `ACCEL`：直线加速赛分段加速与制动。
4. `SKIDPAD`：八字状态机轨迹。

## 7. 话题建议

1. 统一管线输出（`planner:=unified`）：
   1. `planning/pathlimits` — 单一话题，三赛项共用
2. 兼容输出（仅过渡，不作为主链路）：
   1. `planning/high_speed_tracking/pathlimits/partial`
   2. `planning/high_speed_tracking/pathlimits/full`
3. 新接口新增（V2，后续）：
   1. `planning/high_speed_tracking/pathlimits_v2/full`
   2. `planning/high_speed_tracking/pathlimits_v2/partial`
   3. `planning/line_detection/pathlimits_v2`
   4. `planning/skidpad/pathlimits_v2`

### 7.1 统一管线参数

- `output_pathlimits_topic`：统一节点的输出话题名，默认 `planning/pathlimits`
- `mission`：驱动后端选择的任务参数（`line`/`acceleration`/`skidpad`/`high_speed`/`trackdrive`/`autocross`）

## 8. 降级策略

1. 任一契约校验失败：`replan=true`，并发布保守速度。
2. 速度规划失败：只发布几何轨迹 + 保守限速。
3. 边界可信度不足：缩短规划窗并限制最大速度。

## 9. 迁移验收门槛

1. V2 消息 100% 满足长度一致性校验。
2. 控制侧在 V2 输入下无回归（与 V1 基线对比）。
3. 三赛项都能输出 V2 完整字段。
