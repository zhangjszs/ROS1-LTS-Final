# Vision Python Default and Fusion Default Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make the shared launch chain default to the Python vision node and enable LiDAR vision color injection by default.

**Architecture:** Keep the existing dual implementation design, but move the default path to Python by changing launch argument defaults and threading a `vision_impl` parameter through the shared `fsd_launch` wrappers. Turn on LiDAR-side fusion by flipping the shared `vision_inject.enabled` default in the base perception config.

**Tech Stack:** ROS Noetic launch XML, catkin, YAML configuration

---

### Task 1: Record the default launch chain changes

**Files:**
- Modify: `src/vision_ros/launch/vision.launch`
- Modify: `src/fsd_launch/launch/subsystems/vision.launch`
- Modify: `src/fsd_launch/launch/subsystems/mission_stack.launch`
- Modify: `src/fsd_launch/launch/trackdrive.launch`

**Step 1: Write the failing test**

Config-only change. Use launch inspection as verification instead of adding production tests.

**Step 2: Run test to verify it fails**

Run: `source /opt/ros/noetic/setup.bash && source devel/setup.bash && timeout 12 roslaunch fsd_launch trackdrive.launch`
Expected: current defaults do not start Python vision because `enable_vision` defaults to `false` and no `vision_impl` path exists.

**Step 3: Write minimal implementation**

- Default `impl` to `py` in `vision_ros/launch/vision.launch`.
- Add `impl` arg passthrough in `fsd_launch/launch/subsystems/vision.launch`.
- Add `vision_impl` args in mission entrypoints and pass through to the subsystem include.
- Default `enable_vision` to `true` in shared mission launch entrypoints.

**Step 4: Run test to verify it passes**

Run: `source /opt/ros/noetic/setup.bash && source devel/setup.bash && timeout 12 roslaunch fsd_launch trackdrive.launch`
Expected: launch summary shows the vision subsystem enabled by default and the Python node selected unless explicitly overridden.

**Step 5: Commit**

Skip commit in this dirty worktree unless explicitly requested.

### Task 2: Enable LiDAR vision fusion by default

**Files:**
- Modify: `src/perception_ros/config/lidar_base.yaml`

**Step 1: Write the failing test**

Config-only change. Use parameter inspection as verification instead of adding production tests.

**Step 2: Run test to verify it fails**

Run: `source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch perception_ros lidar_cluster.launch`
Expected: current param `vision_inject/enabled` resolves to `false` from the base config.

**Step 3: Write minimal implementation**

- Set `vision_inject.enabled: true` in the shared LiDAR base config.

**Step 4: Run test to verify it passes**

Run: `source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch perception_ros lidar_cluster.launch`
Expected: resolved param `vision_inject/enabled` is `true`.

**Step 5: Commit**

Skip commit in this dirty worktree unless explicitly requested.

### Task 3: Full verification

**Files:**
- Verify only

**Step 1: Write the failing test**

Use the requested end-to-end verification commands.

**Step 2: Run test to verify it fails**

Run: `source /opt/ros/noetic/setup.bash && catkin build vision_ros perception_ros fsd_launch --no-status --summarize`
Expected: captures any launch/config regressions after the default changes.

**Step 3: Write minimal implementation**

No additional implementation if earlier tasks pass.

**Step 4: Run test to verify it passes**

Run:
- `source /opt/ros/noetic/setup.bash && catkin build vision_ros perception_ros fsd_launch --no-status --summarize`
- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && timeout 12 roslaunch fsd_launch trackdrive.launch`

Expected:
- build succeeds
- launch summary shows vision enabled by default
- selected node type is the Python vision node by default

**Step 5: Commit**

Skip commit in this dirty worktree unless explicitly requested.
