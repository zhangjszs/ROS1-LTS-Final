# Track Planning (high_speed_tracking) Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 通过更严格的几何过滤降低 18m+ “墙/护栏”类连续误检对 track 规划的影响，并在同轮回放补齐 cone_map 置信度观测。

**Architecture:** 仅改 `src/planning_ros/config/high_speed_tracking.yaml` 中的过滤参数；回放时录制 `/planning/pathlimits` 与 `/localization/cone_map`，离线脚本输出 JSON 指标用于迭代决策。

**Tech Stack:** ROS Noetic、roslaunch/rosbag、Python3（`rosbag`）离线统计脚本。

---

### Task 1: 实现 cone_map 观测脚本

**Files:**
- Create: `perf_reports/scripts/evaluate_cone_map_metrics.py`

**Step 1: 写脚本（输出 JSON）**
- 输入：`<bag_file>`、`--topic`（默认 `/localization/cone_map`）、`--dist-th`（默认 18）、`--y-th`（默认 1，可选）
- 输出指标（建议）：总锥桶数、dist>=th 的锥桶数、abs(y)<=y_th 且 dist>=th 的锥桶数；各窗口置信度分位数（p10/p50/p90/p95/p99）与直方图（0.0~1.0）。

**Step 2: 快速自检**
- 用本轮回放录下的 cone_map bag 跑一遍，确保能解析自定义消息并生成 JSON。

---

### Task 2: v2 调整 high_speed_tracking 过滤参数

**Files:**
- Modify: `src/planning_ros/config/high_speed_tracking.yaml`

**Step 1: 提高三角形最小内角**
- 增大 `min_triangle_angle`（小步递增，先做保守幅度）。

**Step 2: 收紧中点—外接圆心距离阈值**
- 降低 `max_dist_circum_midPoint`（小步递减，先做保守幅度）。

---

### Task 3: 回放并录制关键话题

**Files:**
- (Artifacts) `/tmp/planning_track_tuning_20260220/candidate_v2_*.bag`

**Step 1: 启动回放（关闭 control）**
- Run:
  - `source /opt/ros/noetic/setup.bash`
  - `source devel/.private/fsd_launch/setup.bash`
  - `LAUNCH_FILE="$(rospack find fsd_launch)/launch/subsystems/mission_stack.launch"`
  - `roslaunch "$LAUNCH_FILE" simulation:=true bag:=/home/kerwin/rosbag/track.bag rate:=2.0 loop:=false launch_rviz:=false launch_viz:=false enable_control:=false`

**Step 2: 同时录制**
- Run:
  - `rosbag record /planning/pathlimits /localization/cone_map -O /tmp/planning_track_tuning_20260220/candidate_v2_topics.bag`

---

### Task 4: 计算 proxy 并和 v1/baseline 对比

**Files:**
- (Artifacts) `/tmp/planning_track_tuning_20260220/candidate_v2_planning_metrics.json`
- (Artifacts) `/tmp/planning_track_tuning_20260220/candidate_v2_conemap_metrics.json`

**Step 1: 规划 proxy**
- Run: `python3 perf_reports/scripts/evaluate_planning_metrics.py /tmp/planning_track_tuning_20260220/candidate_v2_pathlimits.bag -o /tmp/planning_track_tuning_20260220/candidate_v2_planning_metrics.json`

**Step 2: cone_map proxy**
- Run: `python3 perf_reports/scripts/evaluate_cone_map_metrics.py /tmp/planning_track_tuning_20260220/candidate_v2_conemap.bag -o /tmp/planning_track_tuning_20260220/candidate_v2_conemap_metrics.json --dist-th 18 --y-th 1`

**Step 3: 比较结论**
- 重点看：`empty_path_rate`、`spike_jump_rate`、`max_curvature` 是否不回退；cone_map 在 dist>=18（以及 abs(y)<=1）窗口的置信度分布是否显示“可安全加阈值”的空间。

