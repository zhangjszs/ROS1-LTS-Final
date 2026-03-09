# Vision Latency Fast Path Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a latency-first fast path in `vision_node_py` that skips ONNX inference for frames already superseded by a newer pending image.

**Architecture:** Keep the current latest-frame worker model. Add one decision point before ONNX inference, route overloaded frames through fallback-only publish, expose one new diagnostic counter, and verify the effect with the existing replay report runner.

**Tech Stack:** Python 3, ROS Noetic (`rospy`, `sensor_msgs`, `diagnostic_msgs`), OpenCV, ONNX Runtime, `unittest`

---

### Task 1: Add regression tests for the latency-first decision

**Files:**
- Modify: `src/vision_ros/test/test_backlog_runtime.py`

**Step 1: Write the failing tests**

Add tests that verify:
- when a newer frame is pending and fallback is enabled, `_process_frame()` must not call `_detect_onnx()`
- when no newer frame is pending, `_process_frame()` still calls `_detect_onnx()`

**Step 2: Run test to verify it fails**

Run: `python3 -m unittest src.vision_ros.test.test_backlog_runtime`

Expected: FAIL because the current implementation still enters `_detect_onnx()` before checking `newer_pending`.

**Step 3: Write minimal implementation**

Introduce the pre-inference overload check in `vision_node_py.py`.

**Step 4: Run test to verify it passes**

Run: `python3 -m unittest src.vision_ros.test.test_backlog_runtime`

Expected: PASS

### Task 2: Add diagnostics for the new fast path

**Files:**
- Modify: `src/vision_ros/scripts/vision_node_py.py`

**Step 1: Write the failing test**

Extend the regression test to assert the new counter increments when inference is skipped due to a newer pending frame.

**Step 2: Run test to verify it fails**

Run: `python3 -m unittest src.vision_ros.test.test_backlog_runtime`

Expected: FAIL because the counter does not exist yet.

**Step 3: Write minimal implementation**

Add:
- `skipped_inference_newer_pending`
- diagnostics KV export for that counter

**Step 4: Run test to verify it passes**

Run: `python3 -m unittest src.vision_ros.test.test_backlog_runtime`

Expected: PASS

### Task 3: Verify package build and replay runner contract

**Files:**
- No new files

**Step 1: Run focused verification**

Run:
- `catkin build vision_ros --no-status --summarize`
- `python3 scripts/test_run_replay_reports.py`

Expected: both pass

### Task 4: Re-run replay evidence with unchanged metrics

**Files:**
- Generated under `perf_reports/data/`

**Step 1: Run the standard replay runner**

Run:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 scripts/run_replay_reports.py --outdir perf_reports/data
```

**Step 2: Inspect the generated reports**

Check:
- `perf_reports/data/vision_backlog_probe_opt2_<tag>/vision_backlog_findings.md`
- `perf_reports/data/legacy_timing_review_opt2_<tag>/timing_review_findings.md`

Expected:
- lower publish lag in overload conditions
- some increase in `normal_merge` frames
- same report schema as before
