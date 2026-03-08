# SKIDPAD_TUNING_TEMPLATE

## 1. 目标

用于 `M4` 新方案（双圆拟合 + 圈次状态机）的 rosbag/实车调参模板，目标是：

1. 快速锁定可用几何拟合参数。
2. 保证四圈相位顺序稳定。
3. 在稳定前提下逐步提高速度参数。

## 2. 启动方式

默认（balanced）：

```bash
roslaunch planning_ros play_skidpad_bag.launch \
  bag:=/path/to/your.bag \
  launch_control:=false
```

切换 profile：

```bash
roslaunch planning_ros play_skidpad_bag.launch \
  bag:=/path/to/your.bag \
  launch_control:=false \
  profile_file:=$(rospack find planning_ros)/config/skidpad_profiles/conservative.yaml
```

`track.bag` 基线 profile：

```bash
roslaunch planning_ros play_skidpad_bag.launch \
  bag:=/home/kerwin/rosbag/track.bag \
  launch_control:=false \
  profile_file:=$(rospack find planning_ros)/config/skidpad_profiles/trackbag_baseline.yaml
```

可选 profile：

1. `src/planning_ros/config/skidpad_profiles/conservative.yaml`
2. `src/planning_ros/config/skidpad_profiles/balanced.yaml`
3. `src/planning_ros/config/skidpad_profiles/aggressive.yaml`
4. `src/planning_ros/config/skidpad_profiles/trackbag_baseline.yaml`

## 3. 观测话题

1. `planning/skidpad/phase`（`std_msgs/UInt8`）
2. `planning/skidpad/geometry_valid`（`std_msgs/Bool`）
3. `planning/skidpad/lap_count`（`std_msgs/Int32`）
4. `planning/skidpad/pathlimits_v2`（轨迹 + 速度曲率）

相位编码：

1. `0=ENTRY`
2. `1=RIGHT_WARMUP`
3. `2=RIGHT_TIMED`
4. `3=CROSSOVER`
5. `4=LEFT_WARMUP`
6. `5=LEFT_TIMED`
7. `6=EXIT`
8. `7=FINISHED`

## 4. 三阶段调参流程

### A. 先锁几何（只用 conservative）

调 `fit.*` 与 `filter.*`，目标：

1. `geometry_valid` 大部分时间为 `true`。
2. `phase` 能从 `ENTRY` 进入圆环流程，不频繁回退。

优先调整顺序：

1. `filter/x_min/x_max/y_min/y_max`
2. `fit/min_cones`
3. `fit/min_cluster_cones`
4. `fit/inlier_threshold`
5. `fit/center_distance_tol` 与 `fit/track_width_tol`

### B. 再锁状态机（balanced）

调 `state_machine.*`，目标：

1. 相位顺序严格单调，不跳相位。
2. `lap_count` 最终达到 4。

优先调整顺序：

1. `phase_min_dwell_steps`
2. `phase_switch_distance`
3. `phase_force_advance_steps`
4. `lap_timeout_steps`
5. `circle_completion_margin`

### C. 最后提速（balanced -> aggressive）

调 `speed.*`，目标：

1. 保持相位正确率不下降。
2. 无明显速度抖动。
3. 控制侧不出现超调/振荡。

优先调整顺序：

1. `timed_speed`
2. `warmup_speed`
3. `crossover_speed`
4. `entry_exit_speed`
5. `max_lateral_acc`、`max_accel`、`max_decel`

## 5. 记录模板

每次实验记录：

1. `bag` 文件名与 profile 名。
2. `geometry_valid` 有效率。
3. 相位序列是否完整（0->1->2->3->4->5->6/7）。
4. `lap_count` 是否到 4。
5. 失败场景（丢圈/卡相位/速度抖动）与对应参数改动。

## 6. 回退规则

若出现以下任一情况，立即回退到 `conservative`：

1. `geometry_valid` 长时间为 `false`。
2. 相位顺序异常（跳相位或卡死）。
3. `lap_count` 无法稳定达到 4。
