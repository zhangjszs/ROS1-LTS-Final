# Color Semantics Replay Validation (2026-03-17)

## Scope
- Rule under validation:
  - `RIGHT = BLUE (y < 0)`
  - `LEFT = RED / YELLOW_SMALL / YELLOW_BIG (y > 0)` (compatibility mode)
- Input bags:
  - `track`: `/tmp/color_semantics_fix_track/track_fused.bag`
  - `accel`: `/tmp/color_semantics_fix_accel/accel_fused.bag`
  - `skidpad`: `/tmp/color_semantics_fix_skidpad/skidpad_fused.bag`
- Structured output: `perf_reports/results/color_semantics_replay_validation_20260317.json`

## Results

### Track
- Topic `/perception/fusion/detections`:
  - frames: `1153`
  - detections: `12243`
  - color counts: `BLUE=4949, YELLOW_SMALL=314, YELLOW_BIG=0, RED=6980, NONE=0`
  - consistency:
    - `BLUE -> right`: `0.8149`
    - `LEFT boundary -> left`: `0.7198`
    - overall: `0.7582`
- Topic `/perception/lidar_cluster/detections`:
  - frames: `1153`
  - detections: `12243`
  - color counts: `BLUE=0, YELLOW_SMALL=9333, YELLOW_BIG=2910, RED=0, NONE=0`

### Accel
- Topic `/perception/fusion/detections`:
  - frames: `932`
  - detections: `1889`
  - color counts: `BLUE=1114, YELLOW_SMALL=0, YELLOW_BIG=0, RED=775, NONE=0`
  - consistency:
    - `BLUE -> right`: `1.0`
    - `LEFT boundary -> left`: `1.0`
    - overall: `1.0`
- Topic `/perception/lidar_cluster/detections`:
  - frames: `932`
  - detections: `1887`
  - color counts: `BLUE=0, YELLOW_SMALL=1599, YELLOW_BIG=288, RED=0, NONE=0`

### Skidpad
- Topic `/perception/fusion/detections`:
  - frames: `1683`
  - detections: `95810`
  - color counts: `BLUE=53738, YELLOW_SMALL=522, YELLOW_BIG=0, RED=41550, NONE=0`
  - consistency:
    - `BLUE -> right`: `0.9944`
    - `LEFT boundary -> left`: `0.9693`
    - overall: `0.9834`
- Topic `/perception/lidar_cluster/detections`:
  - frames: `1683`
  - detections: `95847`
  - color counts: `BLUE=0, YELLOW_SMALL=45671, YELLOW_BIG=50176, RED=0, NONE=0`

## Conclusion
- Replay now records fused stream in all 3 modes (`track/accel/skidpad`).
- Side-consistency improved vs 2026-03-15 baseline (track 0.4925 -> 0.7582).
- Remaining P0 acceptance work:
  1. Freeze explicit consistency thresholds for gating.
  2. Decide whether `track` consistency target is met or needs further tuning.
