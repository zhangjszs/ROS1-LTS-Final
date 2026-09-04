# PLANNING_M2_AUTOCROSS_DESIGN

## 1. 目标

定义高必赛项（Autocross）规划主线：第一圈稳、后续圈快，CPU-only，可输出完整控制轨迹字段。

## 2. 现状证据与切入点

1. 输入：
   1. 锥桶地图：`localization/cone_map`。见 `src/planning_ros/config/high_speed_tracking.yml:2`。
   2. 车身状态：`localization/car_state`。见 `src/planning_ros/config/high_speed_tracking.yml:3`。
2. 当前主链路：
   1. `DelaunayTri::compute(nodes)` 生成三角剖分。见 `src/planning_ros/src/high_speed_tracking/main.cpp:212`。
   2. `wayComputer->update(triangles, stamp)` 生成路径。见 `src/planning_ros/src/high_speed_tracking/main.cpp:217`。
   3. 发布 `HUAT_PathLimits`。见 `src/planning_ros/src/high_speed_tracking/main.cpp:305`。
3. 当前模式差异：
   1. 回环前发 partial，回环后发 full。见 `src/planning_ros/src/high_speed_tracking/main.cpp:228`。
   2. 尚无显式“SAFE_LAP/FAST_LAP”速度剖面切换。

## 3. 赛道约束映射

| 约束来源 | 约束条目 | 设计映射 |
|---|---|---|
| `TRACK_SPECS` | 最小宽度 3.5m | 左右边界配对硬约束 |
| `TRACK_SPECS` | 发夹弯/大曲率 | 曲率上限约束 |
| `TRACK_SPECS` | 蛇形间距 7.62-12.19m | 连边距离先验 |
| `track_constraints.hpp` | 左红右蓝 | 颜色一致性约束 |
| `track_constraints.hpp` | 参考均速 40-48km/h | SAFE_LAP 初始速度上界 |

证据：`docs/competition/traffic_cone.md:15`、`docs/competition/traffic_cone.md:16`、`src/planning_core/include/planning_core/track_constraints.hpp:30`、`src/planning_core/include/planning_core/track_constraints.hpp:62`。

## 4. 算法架构

1. Boundary Graph（边界图层）：
   1. 使用颜色一致性 + 几何先验生成左右边界图。
   2. 未知颜色/错色做降权，不做硬删除。
2. Corridor（走廊层）：
   1. 边界内缩安全裕度，构建可行驶走廊。
3. Centerline Optimizer（轨迹几何层）：
   1. 目标：平滑、低曲率、离边界远。
   2. 约束：轨迹点必须在走廊内。
4. Speed Profile（纵向层）：
   1. 曲率限速。
   2. 前向后向加减速约束。
5. Exporter（接口层）：
   1. 输出 V1（兼容）。
   2. 输出 V2（完整）。

## 5. 模式状态机

1. `MAP_BUILD_SAFE`：
   1. 入口：默认起跑。
   2. 策略：更大安全裕度、较低速度上界。
2. `FAST_LAP`：
   1. 入口：回环状态稳定超过阈值帧数。
   2. 策略：速度上界放宽，向时间最优靠近。
3. 防抖：
   1. 回环判定必须连续 K 帧成立。
   2. 模式切换后最短保持 L 帧。

## 6. 伪代码

```pseudo
input: cone_map, car_state

cones = filter_confidence(cone_map)
if cones is empty:
  publish_fallback()
  return

boundary_graph = build_boundary_graph(
  cones,
  color_rule = {red:left, blue:right},
  width_min = 3.5
)

corridor = build_corridor(boundary_graph, safety_margin)

centerline_seed = delaunay_midpoint_seed(cones)
centerline = optimize_centerline(
  seed = centerline_seed,
  objective = smooth + low_curvature + boundary_clearance,
  constraints = inside(corridor)
)

s, yaw, kappa = geometric_profile(centerline)
v_lat = sqrt(a_y_max / max(abs(kappa), eps))
v_raw = min(v_lat, v_mode_limit)
v_ref = forward_backward_pass(v_raw, a_acc_max, a_brake_max)

if mode == MAP_BUILD_SAFE:
  v_ref *= alpha_safe
else if mode == FAST_LAP:
  v_ref *= alpha_fast

t_ref = integrate_time(s, v_ref)
a_ref = differentiate(v_ref, t_ref)

publish_v1(path, tracklimits, replan)
publish_v2(path, s, yaw, kappa, v_ref, a_ref, t_ref, profile_mode)
```

## 7. 参数建议

1. `boundary.width_min = 3.5`
2. `boundary.color_mismatch_penalty`
3. `corridor.safety_margin`
4. `optimizer.curvature_weight`
5. `speed.a_y_max`
6. `speed.a_acc_max`
7. `speed.a_brake_max`
8. `mode.alpha_safe`
9. `mode.alpha_fast`
10. `mode.switch_debounce_frames`

## 8. 验证计划

1. 安全：
   1. 出界率。
   2. 最小锥桶距离违规率。
2. 动态：
   1. 曲率超限率。
   2. 加减速度超限率。
3. 稳定性：
   1. 轨迹点跳变率。
   2. replan 抖动频率。
4. 赛项表现：
   1. 第一圈闭环成功率。
   2. 后续圈平均圈速提升率。

## 9. 与当前实现差异

1. 当前以 Delaunay 中点搜索为主，新增边界图与走廊优化层。
2. 当前无完整速度时标，新增 `s/yaw/time/accel` 输出。
3. 当前无显式模式状态机，新增 `SAFE_LAP/FAST_LAP`。

## 10. 统一管线中的行为等价性

在 `PlanningPipelineNode` 中，`mission=high_speed`（或 `trackdrive`/`autocross`）分支的行为与旧 `high_speed_tracking_node` 完全等价：
- `callback_ccat` → `HighSpeedConeCallback` 成员方法
- `finish()` → `HighSpeedFinishCheck()` 成员方法
- 全局变量（`wasLoopClosed`、`interTimes` 等）→ 类成员变量
- Visualization 单例和 PerfStats 保留
- SAFE_LAP/FAST_LAP 切换逻辑不变（回环判定 + fallback by lap counter）

A/B 对比验证：
```bash
# 旧节点
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=... planner:=high_speed
# 新节点
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=... planner:=unified
```
