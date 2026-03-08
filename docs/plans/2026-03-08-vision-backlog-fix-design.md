# Vision Backlog Fix Design

## Goal

Eliminate multi-second stale `vision/detections` publishes in `vision_node_py` so legacy fusion can consume fresh camera detections again.

## Scope

- Modify only `src/vision_ros/scripts/vision_node_py.py`
- Keep adapter, fusion budget, TF, and calibrated mode unchanged
- Add minimal regression tests for backlog control and stage timing
- Re-run the existing timing review reports without changing their metrics

## Problem

Runtime evidence shows:

- `vision` header stamps are close to LiDAR stamps
- `vision` publishes detections several seconds after those header stamps
- `vision` output cadence is around `2 Hz` while input image cadence is much higher in `trackdrive`

This means the node is processing old frames, not fresh ones.

## Design

### Processing Model

Replace the current callback-thread processing model with a latest-frame worker model:

- Subscriber callback stores only the newest `sensor_msgs/Image`
- Any older pending frame is overwritten and counted as replaced
- A worker thread wakes up, takes the newest pending frame, and processes it
- Frames older than a configurable monotonic-age threshold are dropped before processing

This preserves freshness and bounds backlog without changing detector logic.

### Stage Timing

Add per-frame timing capture for:

- `receive_to_pick_ms`
- `preprocess_ms`
- `inference_stage_ms`
- `postprocess_ms`
- `publish_ms`
- `total_processing_ms`
- `end_to_end_publish_lag_ms`

Expose the latest timing snapshot and drop counters through vision diagnostics.

### Configuration

Add a minimal stale-frame threshold parameter:

- `~node/stale_frame_age_sec`

Default should be conservative but sub-second so the node favors freshness over completeness.

## Testing

Add narrow Python regression tests for:

- latest-frame overwrite behavior
- stale-frame drop behavior
- stage timing snapshot calculation

Then re-run:

- `timing_review_findings.md`
- `vision_backlog_findings.md`

using the same commands and metrics as before.
