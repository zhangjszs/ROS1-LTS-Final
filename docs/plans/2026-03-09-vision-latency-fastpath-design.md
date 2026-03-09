# Vision Latency Fast Path Design

## Goal

Reduce `vision_node_py` end-to-end publish lag by avoiding full ONNX inference on frames that are already stale relative to a newer pending image.

## Scope

- Modify only `src/vision_ros/scripts/vision_node_py.py`
- Reuse the existing latest-frame worker model and diagnostics pipeline
- Add narrow regression tests in `src/vision_ros/test/test_backlog_runtime.py`
- Re-run the existing replay report suites without changing report format

## Problem

Latest replay evidence shows:

- `receive_to_pick_ms` is small enough in `trackdrive` and moderate in `skidpad`
- `preprocess_ms`, `postprocess_ms`, and `publish_ms` are not the dominant cost
- `inference_stage_ms` is consistently around `470-480 ms`

Current round-2 logic checks `newer_pending` only after inference. That saves tracker/debug work, but it does not remove the dominant latency source.

## Design

### Latency-First Fast Path

When a newer frame is already pending before ONNX begins, the current frame should skip ONNX and publish a fast fallback result instead of spending another `~0.5 s` on stale inference.

Rules:

- Only activate this path when `skip_heavy_if_newer_pending=true`
- Only activate it when `fallback_enabled=true`
- Detect `newer_pending` before ONNX starts
- If active, skip `_detect_onnx()` entirely for that frame
- Continue using fallback HSV detections plus the normal publish path

This keeps output fresh at the cost of model accuracy on overloaded frames, which matches the current priority.

### Diagnostics

Add a dedicated counter and timing visibility for the new behavior:

- `skipped_inference_newer_pending`

Keep the existing stage timing keys unchanged so replay reports remain comparable.

## Expected Outcome

This will not make ONNX faster. It will reduce publish lag only in overload conditions by refusing to spend heavy inference time on a frame that has already been superseded.

The success criteria for this round are:

- lower `end_to_end_publish_lag_ms`
- more `normal_merge` frames in `decision/trace`
- no regression in adapter/runtime correctness
