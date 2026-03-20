# Color Semantics Replay Validation (2026-03-15)

## Scope
- Rule under validation:
  - `RIGHT = BLUE (y < 0)`
  - `LEFT = RED / YELLOW_SMALL / YELLOW_BIG (y > 0)` (compatibility mode)
- Input bags:
  - `track`: `/tmp/color_semantics_track/track_fused.bag`
  - `accel`: `/tmp/color_semantics_accel/accel_fused.bag`
  - `skidpad`: `/tmp/color_semantics_skidpad/skidpad_fused.bag`
- Structured output: `perf_reports/results/color_semantics_replay_validation_20260315.json`

## Results

### Track
- Topic `/perception/fusion/detections`:
  - frames: `443`
  - detections: `4832`
  - color counts: `BLUE=1274, YELLOW_SMALL=1328, YELLOW_BIG=724, RED=1506, NONE=0`
  - consistency:
    - `BLUE -> right`: `0.5581`
    - `LEFT boundary -> left`: `0.4691`
    - overall: `0.4925`
- Topic `/perception/lidar_cluster/detections`:
  - frames: `442`
  - detections: `4822`
  - color counts: `BLUE=0, YELLOW_SMALL=3676, YELLOW_BIG=1146, RED=0, NONE=0`

### Accel
- Topic `/perception/fusion/detections`:
  - frames: `0` (no fused messages recorded)
- Topic `/perception/lidar_cluster/detections`:
  - frames: `443`
  - detections: `879`
  - color counts: `BLUE=0, YELLOW_SMALL=742, YELLOW_BIG=137, RED=0, NONE=0`

### Skidpad
- Topic `/perception/fusion/detections`:
  - frames: `0` (no fused messages recorded)
- Topic `/perception/lidar_cluster/detections`:
  - frames: `492`
  - detections: `27307`
  - color counts: `BLUE=0, YELLOW_SMALL=13678, YELLOW_BIG=13629, RED=0, NONE=0`

## Conclusion
- Code-level color semantic unification is in place, but replay-level acceptance is **not closed**:
  - `track` fused stream exists, yet boundary-side consistency is about `49.25%`.
  - `accel/skidpad` produced no fused stream in this replay window, so RED/BLUE semantic chain was not exercised end-to-end.
- Remaining P0 acceptance work:
  1. Ensure fused stream is present in `accel/skidpad` replay.
  2. Improve side-consistency KPI and freeze threshold for gate usage.
