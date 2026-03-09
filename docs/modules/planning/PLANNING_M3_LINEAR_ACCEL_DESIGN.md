# PLANNING_M3_LINEAR_ACCEL_DESIGN

## 1. 文档目的

定义直线加速赛项（Linear Acceleration）的规划设计方案，满足：

1. CPU-only，不依赖深度网络。
2. 兼容当前 `line_detection` 节点输入输出。
3. 输出可用于控制的完整速度/曲率轨迹（通过 `HUAT_PathLimitsV2`）。

本设计为计划阶段产物，仅包含设计、接口、伪代码和测试计划，不涉及代码修改。

## 2. 现状接口证据

1. 当前输入为锥桶检测与车辆状态同步：
   1. `topics/cone = perception/lidar_cluster/detections`
   2. `topics/car_state = localization/car_state`
   3. 证据：`src/planning_ros/src/line_detection_node.cpp:48`
2. 当前输出为：
   1. 路径 `planning/line_detection/path` (`nav_msgs/Path`)
   2. 完成信号 `planning/line_detection/finish_signal` (`std_msgs/Bool`)
   3. 证据：`src/planning_ros/src/line_detection_node.cpp:50`
3. 当前算法链路：
   1. Hough 变换检测双边线
   2. 取中心线并离散生成路径
   3. 世界坐标转换
   4. 终点检测
   5. 证据：`src/planning_core/src/line_detection_core.cpp:71`、`src/planning_core/src/line_detection_core.cpp:176`、`src/planning_core/src/line_detection_core.cpp:237`、`src/planning_core/src/line_detection_core.cpp:266`

## 3. 赛道约束映射（来自 TRACK_SPECS）

| 约束来源 | 约束条目 | M3 映射 |
|---|---|---|
| `TRACK_SPECS` | 赛道宽度约 3.0m | 双边线间距约束，中心线必须位于两边界中间 |
| `TRACK_SPECS` | 起跑/计时线间距 0.3m | `start_offset_s = 0.3m`，计时起点从弧长坐标偏置 |
| `TRACK_SPECS` | 加速区长度 75m | 速度规划第一段加速目标区间 |
| `TRACK_SPECS` | 制动区长度 100m | 速度规划第二段制动到 0 |
| `TRACK_SPECS` | 锥桶间距 5m | 边界线点关联距离先验和缺失补全窗口 |

证据：`src/planning_core/TRACK_SPECS.md:11`、`src/planning_core/TRACK_SPECS.md:12`、`src/planning_core/TRACK_SPECS.md:13`、`src/planning_core/TRACK_SPECS.md:14`、`src/planning_core/TRACK_SPECS.md:15`。

## 4. 目标与约束

### 4.1 硬约束

1. 不出界：路径必须位于两侧边界线之间并保留安全裕度。
2. 不撞锥：中心线到边界最小距离大于安全距离。
3. 动态可行：速度和加速度满足车辆约束。
4. 完赛逻辑：通过 75m 终点后进入 100m 制动段，最终速度收敛到 0。

### 4.2 可优化目标

1. 更稳定的边界估计（噪声/缺锥情况下路径不跳变）。
2. 更平顺的纵向速度曲线（低冲击制动）。

## 5. 输入输出接口设计

### 5.1 输入

1. 锥桶：`perception/lidar_cluster/detections`。
2. 车辆状态：`localization/car_state`。
3. 坐标要求：锥桶 `base_link/velodyne`，输出 `world`。
4. 证据：`src/planning_ros/src/line_detection_node.cpp:53`。

### 5.2 输出

1. 兼容输出（保留）：
   1. `nav_msgs/Path` 到 `planning/line_detection/path`
   2. `std_msgs/Bool` 到 `planning/line_detection/finish_signal`
2. 新增输出（建议）：
   1. `HUAT_PathLimitsV2` 到 `planning/line_detection/pathlimits_v2`
   2. `profile_mode = ACCEL`

## 6. 算法方案（M3）

### 6.1 边界与中心线构建

1. 预处理：
   1. 距离门限过滤（沿用当前参数：`min/max_distance`）。
   2. 侧向门限过滤（沿用 `max_cone_lateral_distance`）。
2. 双边界估计：
   1. 主方法：鲁棒并行直线拟合（RANSAC + 最小二乘精修）。
   2. 备用：当前 Hough 结果（失败时回退）。
3. 中心线生成：
   1. 由左右边界中线直接生成。
   2. 按固定步长离散（沿用 `path/interval`）。

### 6.2 纵向速度规划（核心）

1. 路径弧长化，计算 `s[i]`。
2. 分段目标：
   1. `0 <= s < 75m`：加速段，`v_ref` 受 `a_acc_max` 约束增加。
   2. `75m <= s < 175m`：制动段，`v_ref` 受 `a_brake_max` 约束减小至 0。
   3. `s >= 175m`：保持 0。
3. 曲率影响：
   1. 直线任务曲率接近 0，但仍输出 `curvatures[]` 供控制统一处理。
4. 时标化：
   1. 由 `s,v_ref` 积分得到 `target_time[]`。
   2. 差分得到 `target_accels[]`。

### 6.3 完赛与终点触发

1. 当前终点检测逻辑以 `vehicle_state.x` 对比阈值，建议扩展为弧长触发。
2. 保留 `finish_signal` 兼容，同时在 V2 中通过 `profile_mode + v_ref->0`体现进入制动段。

## 7. 伪代码

```pseudo
input: cone_detections, car_state

cones = preprocess(cone_detections)
if cones insufficient:
  publish_replan_or_hold()
  return

# boundary estimation
(left_line, right_line, ok) = robust_parallel_fit(cones)
if not ok:
  (left_line, right_line, ok) = hough_fallback(cones)
if not ok:
  publish_replan_or_hold()
  return

centerline = build_centerline(left_line, right_line, step = path_interval)
centerline_world = transform_to_world(centerline, car_state, imu_offset)

s = cumulative_arc_length(centerline_world)
yaw = heading_profile(centerline_world)
kappa = curvature_profile(centerline_world)

for each i:
  if s[i] < 75.0:
    v_ref[i] = accel_limited_profile(i)
  else if s[i] < 175.0:
    v_ref[i] = braking_profile_to_zero(i)
  else:
    v_ref[i] = 0.0

t_ref = integrate_time(s, v_ref)
a_ref = finite_difference(v_ref, t_ref)

publish_nav_path(centerline_world)
publish_finish_if_needed(s, car_state)
publish_pathlimits_v2(path, s, yaw, kappa, v_ref, a_ref, t_ref, mode=ACCEL)
```

## 8. 参数建议（规划层）

1. `accel.start_offset_s = 0.3`
2. `accel.length = 75.0`
3. `brake.length = 100.0`
4. `track.width_nominal = 3.0`
5. `safety.margin = 0.25`（示例，待实车标定）
6. `speed.a_acc_max`（待车辆动力学标定）
7. `speed.a_brake_max`（待车辆制动标定）
8. `speed.jerk_limit`（可选）

## 9. 验证计划（M3）

### 9.1 功能正确性

1. 轨迹始终在左右边界内。
2. 终点后进入制动阶段并在 100m 内降到 0。
3. `finish_signal` 与 `s=75m` 事件一致。

### 9.2 接口正确性

1. `path` 与各数组长度完全一致。
2. `target_time` 严格递增。
3. 所有数值有限且单位正确。

### 9.3 稳定性

1. 锥桶缺失 10%/20% 情况下仍可输出可跟踪路径。
2. 时戳错位场景下（保留当前同步阈值）输出不抖动。

### 9.4 指标

1. 出界率。
2. 撞锥率。
3. 完赛成功率。
4. 路径横向抖动（中心线方差）。
5. 速度剖面平滑性（加速度变化率）。

## 10. 与现有实现差异（计划项）

1. 现状仅输出 `nav_msgs/Path` 和 `finish_signal`，M3 增加 `HUAT_PathLimitsV2`。
2. 现状终点判断主要基于 `x`，M3 推荐基于弧长 `s` 更稳健。
3. 现状无纵向速度规划，M3 明确加速段/制动段目标。

## 11. 统一管线中的用法

在 `PlanningPipelineNode` 中，`mission=line` 或 `mission=acceleration` 分支直接实例化 `LineDetectionNode`，行为与旧独立节点等价。

- 速度剖面通过共享模块 `planning_core::ComputeSpeedProfile` 生成
- 加速/制动区 overlay 参数通过 `line_detection.yaml` 中的 `accel_zone_length` / `brake_zone_length` 配置
- 输出话题由 `output_pathlimits_topic` 参数统一为 `planning/pathlimits`
