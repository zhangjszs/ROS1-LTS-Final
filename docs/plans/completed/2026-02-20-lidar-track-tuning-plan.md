# LiDAR Track Mode (18m+) Tuning Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Reduce 18m+ wall/guardrail false positives while recovering far-range cone recall in Track mode by tuning overlay parameters and validating via rosbag replay.

**Architecture:** Change only `perception_ros` Track overlay YAML. Validate by replaying a known bag, recording `/perception/lidar_cluster/detections`, then computing proxy metrics plus a wall-FP proxy from the recorded detections bag.

**Tech Stack:** ROS Noetic, catkin, roslaunch, rosbag, Python3, `perf_reports/scripts/evaluate_perception_metrics.py`.

---

### Task 1: Prepare baseline capture

**Files:**
- Modify: (none)

**Step 1: Ensure workspace overlays are available**

Run:
- `source /opt/ros/noetic/setup.bash`
- `cd /home/kerwin/2025huat && source devel/.private/perception_ros/setup.bash`

Expected: `rospack find perception_ros` succeeds.

**Step 2: Replay track bag and record detections (baseline)**

Run:
- `export OUT_DIR=/tmp/lidar_track_tuning`
- `mkdir -p "$OUT_DIR"`
- `rosparam set use_sim_time true`
- Start replay in one terminal:
  - `roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/home/kerwin/rosbag/track.bag rate:=2.0 loop:=false launch_rviz:=false launch_viz:=false`
- Start record in another terminal:
  - `rosbag record /perception/lidar_cluster/detections -O "$OUT_DIR/baseline_detections.bag"`

Expected:
- Bag file is created and contains `/perception/lidar_cluster/detections`.

**Step 3: Compute baseline metrics**

Run:
- `python3 perf_reports/scripts/evaluate_perception_metrics.py "$OUT_DIR/baseline_detections.bag" -o "$OUT_DIR/baseline_metrics.json"`

Expected: JSON produced with metrics.

**Step 4: Compute baseline wall-FP proxy**

Run:
- `python3 scripts/m5_ab_test.py --help` (if used) OR run an ad-hoc python snippet to compute:
  - `frac(dist>=18 & abs(y)>=6)` over all detections.

Expected: baseline proxy number printed and saved to a text/json file.

---

### Task 2: Apply Track overlay parameter changes (v1)

**Files:**
- Modify: `src/perception_ros/config/lidar_track.yaml`

**Step 1: Strengthen far obstacle-height filtering**

Set (Track overlay):
- `filters/obstacle_height/max_z_span: 0.7`
- `filters/obstacle_height/min_points_to_judge: 8`

**Step 2: Tighten 15–35m cluster tolerance**

Override:
- `mode_presets/track/cluster/cluster_tolerance` to `[0.15, 0.3, 0.5, 0.4, 0.35, 0.3, 0.25]`

**Step 3: Narrow ROI earlier (lateral)**

Override:
- `mode_presets/track/adaptive_y/ramp_start_x: 8.0`
- `mode_presets/track/adaptive_y/far_y_half: 3.0`
(keep `near_y_half: 10.0`)

---

### Task 3: Re-run replay and compare

**Files:**
- Modify: (none)

**Step 1: Replay and record detections (candidate)**

Run:
- same replay command as baseline
- `rosbag record /perception/lidar_cluster/detections -O "$OUT_DIR/candidate_v1_detections.bag"`

**Step 2: Compute candidate metrics**

Run:
- `python3 perf_reports/scripts/evaluate_perception_metrics.py "$OUT_DIR/candidate_v1_detections.bag" -o "$OUT_DIR/candidate_v1_metrics.json"`

**Step 3: Compute candidate wall-FP proxy**

Run:
- same proxy computation, output to `$OUT_DIR/candidate_v1_wall_fp.json` (or `.txt`)

**Step 4: Decide next iteration**

If wall-FP proxy improved but recall dropped:
- loosen `cluster_tolerance` slightly in 25–35m (e.g. `0.35 -> 0.38`) or reduce `min_confidence_far` slightly.

If wall-FP proxy unchanged:
- decrease `filters/obstacle_height/grid_size` (e.g. `0.5 -> 0.35`) OR decrease `max_z_span` back down (treat more cells as “tall obstacle”) and re-test.
