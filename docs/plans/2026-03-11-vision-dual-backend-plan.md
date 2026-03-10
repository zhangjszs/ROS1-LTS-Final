# Vision Dual Backend Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make `vision_node_py` support both the existing ONNX model path and direct YOLO `.pt` inference, while fixing parsing for the checked-in `yolo26l_48g` ONNX export.

**Architecture:** Keep the current single-node worker pipeline. Add backend-specific initialization and detection helpers, normalize both backends into `Detection`, and keep fusion, tracking, diagnostics, and ROS publishing unchanged.

**Tech Stack:** Python 3, ROS Noetic (`rospy`, `sensor_msgs`, `diagnostic_msgs`), OpenCV, ONNX Runtime, PyTorch, Ultralytics YOLO, `unittest`

---

### Task 1: Add regression tests for ONNX end-to-end output parsing

**Files:**
- Modify: `src/vision_ros/test/test_backlog_runtime.py`

**Step 1: Write the failing tests**

Add tests that verify `_detect_onnx()` can parse:
- `[1, N, 6]` output for end-to-end detections
- `[N, 6]` output for end-to-end detections

Each test should assert:
- confidence threshold filtering works
- class IDs are preserved
- `class_to_color` mapping is applied
- bounding boxes are scaled from model input size back to source image size

**Step 2: Run test to verify it fails**

Run:

```bash
source /opt/ros/noetic/setup.bash && python3 -m unittest src.vision_ros.test.test_backlog_runtime
```

Expected: FAIL because current `_detect_onnx()` only understands YOLO raw tensor outputs.

**Step 3: Write minimal implementation**

Update `_detect_onnx()` and add a helper for `[N, 6]` end-to-end outputs.

**Step 4: Run test to verify it passes**

Run:

```bash
source /opt/ros/noetic/setup.bash && python3 -m unittest src.vision_ros.test.test_backlog_runtime
```

Expected: PASS for the new ONNX regression cases.

### Task 2: Add regression tests for `yolo_pt` backend initialization and conversion

**Files:**
- Modify: `src/vision_ros/test/test_backlog_runtime.py`

**Step 1: Write the failing tests**

Add tests that verify:
- `_init_backend()` accepts `backend_type=yolo_pt` when `ultralytics` is importable and the model path ends with `.pt`
- `_detect_yolo_pt()` converts YOLO result boxes into `Detection` objects
- mismatched backend/model extensions are rejected when `require_model=true`

**Step 2: Run test to verify it fails**

Run:

```bash
source /opt/ros/noetic/setup.bash && python3 -m unittest src.vision_ros.test.test_backlog_runtime
```

Expected: FAIL because the node currently rejects any backend other than `onnx` or `fallback_only`.

**Step 3: Write minimal implementation**

Add:
- optional `ultralytics` import handling
- `yolo_pt` initialization path
- PT detection conversion helper
- backend-specific path validation

**Step 4: Run test to verify it passes**

Run:

```bash
source /opt/ros/noetic/setup.bash && python3 -m unittest src.vision_ros.test.test_backlog_runtime
```

Expected: PASS for the new PT regression cases.

### Task 3: Wire config and diagnostics for the dual-backend flow

**Files:**
- Modify: `src/vision_ros/config/vision_base.yaml`
- Add: `src/vision_ros/config/vision_local_yolo26l_48g_onnx.yaml`
- Add: `src/vision_ros/config/vision_local_yolo26l_48g_pt.yaml`
- Modify: `src/vision_ros/scripts/vision_node_py.py`

**Step 1: Write the failing test**

Extend diagnostics-related tests so they assert:
- `backend_name` reports `onnx`, `yolo_pt`, or `fallback_hsv` consistently
- diagnostics expose the selected backend and model path

**Step 2: Run test to verify it fails**

Run:

```bash
source /opt/ros/noetic/setup.bash && python3 -m unittest src.vision_ros.test.test_backlog_runtime
```

Expected: FAIL until the backend branching and config defaults are updated.

**Step 3: Write minimal implementation**

Update:
- backend type comments/config docs
- local overlays for `yolo26l_48g`
- diagnostics/export paths if needed for the new backend

**Step 4: Run test to verify it passes**

Run:

```bash
source /opt/ros/noetic/setup.bash && python3 -m unittest src.vision_ros.test.test_backlog_runtime
```

Expected: PASS with the diagnostics assertions included.

### Task 4: Run focused verification in the ROS workspace

**Files:**
- No new files

**Step 1: Run the focused regression tests**

Run:

```bash
source /opt/ros/noetic/setup.bash && python3 -m unittest src.vision_ros.test.test_backlog_runtime
```

Expected: PASS

**Step 2: Run package-level verification**

Run:

```bash
source /opt/ros/noetic/setup.bash && catkin build vision_ros --no-status --summarize
source /opt/ros/noetic/setup.bash && catkin run_tests vision_ros --no-status --summarize
source /opt/ros/noetic/setup.bash && catkin_test_results build/vision_ros
```

Expected: build succeeds and `vision_ros` tests pass.
