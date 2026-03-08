# Fusion Adapter Hardening Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Harden `cone_detection_adapter` so unified detections follow a deterministic steady-time deadline with finalized tombstones and runtime counters.

**Architecture:** Refactor the adapter into a small state machine keyed by raw header stamp, move waiting logic to steady time, derive the adapter budget from the same runtime fusion timing source, and verify behavior with targeted TDD tests plus a launch/config contract test.

**Tech Stack:** ROS Noetic, C++17, GTest, rostest/python unittest.

---

### Task 1: Add failing adapter timing tests

**Files:**
- Modify: `src/perception_ros/test/test_cone_detection_adapter.cpp`

**Steps:**
1. Add tests for overdue fused rejection, raw-finalized tombstone rejection, and counter increments.
2. Run the adapter test binary or package-local build target to verify the new tests fail.

### Task 2: Refactor the adapter state machine

**Files:**
- Modify: `src/perception_ros/include/perception_ros/cone_detection_adapter.hpp`
- Modify: `src/perception_ros/src/cone_detection_adapter.cpp`

**Steps:**
1. Introduce steady-time deadlines and finalized tombstones.
2. Add observable counters and accessors.
3. Ensure publish paths finalize stamps and refuse late rewrites.
4. Re-run the adapter tests until green.

### Task 3: Update node runtime contract handling

**Files:**
- Modify: `src/perception_ros/nodes/cone_detection_adapter_node.cpp`
- Modify: `src/fsd_launch/launch/subsystems/perception.launch`
- Modify: `src/perception_ros/src/lidar_cluster_ros.cpp`

**Steps:**
1. Derive adapter holdoff from the same runtime budget source used by the legacy fusion path.
2. Revive `vision_inject/max_age_sec` as the legacy timing budget input.
3. Add startup logging and contract mismatch warning.
4. Add throttled periodic stats logging.

### Task 4: Refresh contract tests and docs

**Files:**
- Modify: `src/perception_ros/test/test_mainline_adapter_launch_contract.py`
- Optionally modify: `docs/plans/2026-03-08-fusion-mainline-adapter-design.md`

**Steps:**
1. Update the contract test so launch/config no longer asserts a separate adapter holdoff constant.
2. Document the new timing-source rule and finalized semantics.

### Task 5: Verify

**Files:**
- Test: `src/perception_ros/test/test_cone_detection_adapter.cpp`
- Test: `src/perception_ros/test/test_mainline_adapter_launch_contract.py`

**Steps:**
1. Rebuild `perception_ros` test targets.
2. Run adapter unit tests and launch/config contract tests.
3. Summarize residual gaps before replay work.
