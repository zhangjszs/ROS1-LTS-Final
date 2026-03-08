# Vision Backlog Fix Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace stale-frame serial processing in `vision_node_py` with a latest-frame worker model and add full stage timing instrumentation.

**Architecture:** The ROS subscriber callback becomes a cheap handoff point that stores only the newest image message. A worker thread processes that newest frame, drops stale frames before processing, and publishes timing and drop counters through existing vision diagnostics.

**Tech Stack:** ROS Noetic, `rospy`, Python `threading`, existing `vision_node_py` diagnostics, Python `unittest`

---

### Task 1: Add failing regression tests

**Files:**
- Create: `src/vision_ros/test/test_backlog_runtime.py`
- Create: `src/vision_ros/test/test_backlog_runtime.test`
- Modify: `src/vision_ros/CMakeLists.txt`

**Step 1: Write the failing test**

Write tests for:

- latest pending frame overwrite returns the newest frame
- stale pending frame is dropped when picked too late
- stage timing snapshot returns expected ms values

**Step 2: Run test to verify it fails**

Run: `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.vision_ros.test.test_backlog_runtime`

Expected: FAIL because backlog helper classes and timing helper do not exist yet.

**Step 3: Register the test**

Add a new rostest entry in `src/vision_ros/CMakeLists.txt` so the regression stays in package-local test coverage.

**Step 4: Run test again to confirm the failure is still the intended missing behavior**

Run the same command as Step 2.

**Step 5: Commit**

Do not commit yet in this session.

### Task 2: Implement latest-frame backlog control

**Files:**
- Modify: `src/vision_ros/scripts/vision_node_py.py`

**Step 1: Add minimal helper types**

Add small testable helpers for:

- pending image frame metadata
- latest-frame mailbox with overwrite and stale-drop accounting
- stage timing snapshot calculation

**Step 2: Replace callback processing with handoff**

Change `_image_callback` so it only stores the latest image message and wakeups the worker.

**Step 3: Add worker thread**

Add a worker loop that:

- takes the latest pending frame
- drops frames older than `~node/stale_frame_age_sec`
- runs conversion + detection pipeline on the chosen frame only

**Step 4: Run the backlog unit test**

Run: `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.vision_ros.test.test_backlog_runtime`

Expected: PASS

### Task 3: Add stage timing and diagnostics

**Files:**
- Modify: `src/vision_ros/scripts/vision_node_py.py`

**Step 1: Capture timing**

Measure:

- receive to pick
- preprocess
- inference
- postprocess / tracker
- publish
- total processing
- end-to-end publish lag

**Step 2: Publish counters and timing via diagnostics**

Add the latest timing snapshot and frame drop counters to the existing vision diagnostics payload.

**Step 3: Run the unit test again**

Run: `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.vision_ros.test.test_backlog_runtime`

Expected: PASS

### Task 4: Verify build and runtime behavior

**Files:**
- None

**Step 1: Build affected package**

Run: `source /opt/ros/noetic/setup.bash && catkin build vision_ros --no-status --summarize`

Expected: build succeeds.

**Step 2: Re-run existing timing reports with unchanged methodology**

Re-run:

- `legacy_timing_review`
- `vision_backlog_probe`

using the same topic sets and summary scripts as the pre-fix baseline.

**Step 3: Compare against baseline**

Check that:

- vision publish lag is no longer seconds
- representative missions are no longer `100% no_fused`
- fusion counters grow beyond near-zero

**Step 4: Commit**

Do not commit yet in this session.
