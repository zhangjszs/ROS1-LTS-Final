# Fusion Mainline Adapter Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Route fused cone color semantics into localization and planning mainline inputs using a thin adapter node that outputs standard `HUAT_ConeDetections`.

**Architecture:** Add a small adapter in `perception_ros` that merges raw and fused detection streams by stamp and publishes one downstream `HUAT_ConeDetections` topic. Then wire `fsd_launch` to point localization and line/skidpad planning at that unified topic while preserving raw and fused source topics for debugging.

**Tech Stack:** ROS Noetic, C++17, catkin, GTest, roslaunch XML

---

### Task 1: Add failing adapter unit tests

**Files:**
- Create: `src/perception_ros/test/test_cone_detection_adapter.cpp`
- Modify: `src/perception_ros/CMakeLists.txt`

**Step 1: Write the failing test**

Add tests for:
- `PublishesMergedMessageWhenRawAndFusedShareStamp`
- `PublishesRawAfterHoldoffWhenNoFusedArrives`
- `PublishesMergedWhenFusedArrivesBeforeRaw`
- `FallsBackToRawWhenFusedCountMismatches`

**Step 2: Run test to verify it fails**

Run: `catkin build perception_ros --no-status --summarize --make-args test_cone_detection_adapter`

Expected: build/test target fails because adapter class does not exist yet.

**Step 3: Commit**

Do not commit in this session unless explicitly requested.

### Task 2: Implement adapter core and node

**Files:**
- Create: `src/perception_ros/include/perception_ros/cone_detection_adapter.hpp`
- Create: `src/perception_ros/src/cone_detection_adapter.cpp`
- Create: `src/perception_ros/nodes/cone_detection_adapter_node.cpp`
- Modify: `src/perception_ros/CMakeLists.txt`
- Modify: `src/fsd_common/include/fsd_common/topic_contract.hpp`

**Step 1: Write minimal implementation**

Implement:
- cache raw/fused messages by stamp
- publish merged output when matching pair exists
- publish raw after holdoff
- fallback raw on mismatch
- expose configurable topics and holdoff

**Step 2: Run unit test to verify it passes**

Run: `catkin build perception_ros --no-status --summarize --make-args test_cone_detection_adapter`

Expected: adapter unit test target builds cleanly.

### Task 3: Wire launch remaps through stack entrypoints

**Files:**
- Modify: `src/fsd_launch/launch/subsystems/perception.launch`
- Modify: `src/fsd_launch/launch/subsystems/localization.launch`
- Modify: `src/fsd_launch/launch/subsystems/planning.launch`
- Modify: `src/localization_ros/launch/location.launch`
- Modify: `src/planning_ros/launch/planning_pipeline.launch`

**Step 1: Add unified topic args**

Add args for:
- unified cone output topic
- raw/fused input topics where needed

**Step 2: Launch adapter from perception subsystem**

Start the adapter alongside lidar perception and publish the unified topic.

**Step 3: Point localization and planning at unified topic**

Pass the topic into launch params instead of changing node code defaults.

**Step 4: Run focused inspection**

Run: `rg -n "perception/decision/detections|topics/cone" src/fsd_launch src/localization_ros/launch src/planning_ros/launch`

Expected: localization/planning launch paths show unified topic wiring while code defaults remain raw.

### Task 4: Verify targeted build and summarize risks

**Files:**
- Modify if needed after verification: same as above

**Step 1: Run focused tests**

Run:
- `catkin build perception_ros localization_ros planning_ros fsd_launch --no-status --summarize`
- `catkin run_tests perception_ros --no-status --summarize`
- `catkin_test_results build`

**Step 2: Check launch-side contract**

Run:
- `rg -n "perception/decision/detections" src`

Expected: one adapter output contract and launch consumers wired to it.

**Step 3: Commit**

Do not commit in this session unless explicitly requested.
