# Mission-Specific Fusion Budget Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Thread a single mission-owned `decision_fusion_budget_sec` parameter through the launch stack and map it onto the existing legacy fusion budget without changing adapter semantics.

**Architecture:** Mission entry launches own the initial values. `mission_stack` and `perception` only pass the same arg downward. `lidar_cluster.launch` is the only place that maps the mission arg onto `vision_inject/max_age_sec`.

**Tech Stack:** ROS Noetic launch XML, Python `unittest` launch contract tests.

---

### Task 1: Lock the contract with tests

**Files:**
- Modify: `src/perception_ros/test/test_mainline_adapter_launch_contract.py`

**Step 1:** Add failing tests for mission-entry defaults.
**Step 2:** Add failing tests that `mission_stack` and `perception` pass `decision_fusion_budget_sec` through unchanged.
**Step 3:** Add failing test that `lidar_cluster.launch` maps `decision_fusion_budget_sec` to `vision_inject/max_age_sec`.

### Task 2: Thread the mission-owned budget through launch files

**Files:**
- Modify: `src/fsd_launch/launch/trackdrive.launch`
- Modify: `src/fsd_launch/launch/autocross.launch`
- Modify: `src/fsd_launch/launch/acceleration.launch`
- Modify: `src/fsd_launch/launch/skidpad.launch`
- Modify: `src/fsd_launch/launch/subsystems/mission_stack.launch`
- Modify: `src/fsd_launch/launch/subsystems/perception.launch`
- Modify: `src/perception_ros/launch/lidar_cluster.launch`

**Step 1:** Add mission-specific defaults at entry launches.
**Step 2:** Pass the same arg through `mission_stack` into `perception`.
**Step 3:** Pass the same arg into `lidar_cluster.launch`.
**Step 4:** Set `vision_inject/max_age_sec` from that arg inside the LiDAR cluster node launch wrapper.

### Task 3: Verify defaults and fallback

**Files:**
- Reference: launch files above

**Step 1:** Run contract tests.
**Step 2:** Launch at least one mission entry and check the runtime budget matches the mission default.
**Step 3:** Launch `mission_stack.launch` directly without an explicit mission budget and confirm it falls back to `0.15s`.
