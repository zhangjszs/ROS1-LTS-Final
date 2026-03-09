# PLANNING_M4_SKIDPAD_DESIGN

## 1. 目标

定义八字赛项（Skidpad）规划方案，满足：

1. 严格对齐八字几何与圈次流程约束。
2. 输出可控制的完整轨迹字段（V2）。
3. 保持 CPU-only。

## 2. 现状证据与问题

1. 当前输入输出：
   1. 输入锥桶：`perception/lidar_cluster/detections`。见 `src/planning_ros/config/skidpad_detection.yaml:3`。
   2. 输入车辆状态：`localization/car_state`。见 `src/planning_ros/config/skidpad_detection.yaml:4`。
   3. 输出 `nav_msgs/Path`：`planning/skidpad/log_path`。见 `src/planning_ros/config/skidpad_detection.yaml:5`。
2. 当前核心逻辑：
   1. 仅取筛选后点云前四个点估计姿态。见 `src/planning_core/src/skidpad_detection_core.cpp:51`。
   2. 轨迹采用固定分段直线+圆弧生成。见 `src/planning_core/src/skidpad_detection_core.cpp:238`。
   3. 圆半径硬编码 `9.125`。见 `src/planning_core/src/skidpad_detection_core.cpp:190`。
3. 当前主要缺口：
   1. 未显式对齐 4 圈流程与计时圈切换。
   2. 无 V2 输出字段（速度/曲率/时间）。
   3. 缺少鲁棒圆拟合与边界一致性约束。

## 3. 赛道约束映射

| 约束来源 | 约束条目 | M4 映射 |
|---|---|---|
| `TRACK_SPECS` | 圆半径 15.25m | 双圆边界拟合约束 |
| `TRACK_SPECS` | 圆心距 18.25m | 两圆中心距离约束 |
| `TRACK_SPECS` | 赛道宽度 3.0m | 内外边界差约束 |
| `TRACK_SPECS` | 右1/右2/切换/左3/左4流程 | 圈次状态机 |
| `track_constraints.hpp` | `total_laps=4`, `timed_laps={2,4}` | 计时圈逻辑 |

证据：`src/planning_core/TRACK_SPECS.md:36`、`src/planning_core/TRACK_SPECS.md:37`、`src/planning_core/TRACK_SPECS.md:38`、`src/planning_core/TRACK_SPECS.md:52`、`src/planning_core/include/planning_core/track_constraints.hpp:51`。

## 4. 方案设计

1. 感知预处理：
   1. 使用全部可用锥桶做聚类，不再依赖仅四点。
   2. 保留当前滤波边界作为初始门限，再加 RANSAC 圆拟合。
2. 双圆拟合：
   1. 分别拟合左右圆内外边界。
   2. 检查圆心距与赛道宽度，失败则降级。
3. 参考轨迹生成：
   1. 构建“入场直线 -> 右圆两圈 -> 过渡 -> 左圆两圈 -> 退出”统一轨迹。
   2. 输出连续 `s,yaw,kappa`。
4. 纵向策略：
   1. 计时圈可略高速度。
   2. 过渡段与出场段限制横摆冲击。
5. 输出：
   1. 保留 `nav_msgs/Path` 兼容。
   2. 新增 `HUAT_PathLimitsV2`，`profile_mode = SKIDPAD`。

## 5. 圈次状态机

1. `ENTRY`：入场。
2. `RIGHT_WARMUP`：右圆第 1 圈。
3. `RIGHT_TIMED`：右圆第 2 圈。
4. `CROSSOVER`：右转左过渡。
5. `LEFT_WARMUP`：左圆第 3 圈。
6. `LEFT_TIMED`：左圆第 4 圈。
7. `EXIT`：离场。

切换条件：

1. 基于弧长和圆相位联合判定，不使用单点距离阈值。
2. 每次状态切换要求满足最短驻留时间，抑制抖动。

## 6. 伪代码

```pseudo
input: cone_detections, car_state

cones = preprocess_and_cluster(cone_detections)

circles = fit_skidpad_circles(cones)   # left/right, inner/outer
if not validate_geometry(circles, center_dist=18.25, width=3.0):
  publish_safe_fallback()
  return

phase = update_phase_machine(car_state, circles, lap_counter)

path = generate_phase_reference(phase, circles)
s, yaw, kappa = geometric_profile(path)
v_ref = skidpad_speed_profile(phase, kappa)
t_ref = integrate_time(s, v_ref)
a_ref = differentiate(v_ref, t_ref)

publish_nav_path(path)
publish_v2(path, s, yaw, kappa, v_ref, a_ref, t_ref, profile_mode=SKIDPAD)
publish_approaching_goal_if_needed(phase)
```

## 7. 参数建议

1. `skidpad.fit.ransac_iter`
2. `skidpad.fit.inlier_thresh`
3. `skidpad.geometry.center_distance_tol`
4. `skidpad.geometry.width_tol`
5. `skidpad.speed.v_warmup`
6. `skidpad.speed.v_timed`
7. `skidpad.speed.a_lat_max`
8. `skidpad.phase.min_dwell_time`

## 8. 验证计划

1. 几何一致性：
   1. 拟合圆半径误差。
   2. 圆心距误差。
   3. 赛道宽度误差。
2. 流程一致性：
   1. 四圈顺序识别正确率。
   2. 第 2/4 圈计时标记正确率。
3. 轨迹质量：
   1. 曲率连续性。
   2. 速度剖面平滑性。
4. 鲁棒性：
   1. 缺锥/误检/错色场景下完赛率。

## 9. 与当前实现差异

1. 当前"四点估计 + 固定圆弧"改为"全局圆拟合 + 圈次状态机"。
2. 当前仅 `nav_msgs/Path` 改为 `Path + V2` 双输出。
3. 当前距离阈值切段改为相位和弧长切段。

## 10. 统一管线中的用法

在 `PlanningPipelineNode` 中，`mission=skidpad` 分支直接实例化 `SkidpadDetectionNode`，行为与旧独立节点等价。

- phase-aware speed cap 通过 `GetRecommendedSpeedCap()` 获取，传入共享模块 `ComputeSpeedProfile`
- 输出话题由 `output_pathlimits_topic` 参数统一为 `planning/pathlimits`
