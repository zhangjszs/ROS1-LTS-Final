# Fusion Adapter Hardening Design

## Goal
Make `cone_detection_adapter` timing behavior deterministic on the current legacy default path: late fused frames must never rewrite a published frame, and runtime fallback reasons must be observable.

## Scope
- Keep the existing raw/fused/unified topics.
- Do not expand top-level launch entrypoints.
- Do not switch the default fusion path to calibrated TF/camera-info mode.

## Design
### State model
Each raw stamp becomes a pending frame with a steady-clock deadline (`arrival + holdoff`).
A frame can transition only once:
- `pending` -> `published_fused`
- `pending` -> `published_raw_fallback`
After publish, the stamp enters a short-lived finalized tombstone set. Any later raw/fused message with the same stamp is dropped and counted.

### Time source
Use monotonic steady time for waiting and deadlines. ROS header stamps remain the frame identity only.

### Timing contract
The adapter holdoff should derive from the same timing budget the perception fusion path allows.
For the current legacy default path, use `vision_inject/max_age_sec` as the runtime budget source.
If an explicit adapter holdoff override is set and is smaller than the fusion timing budget, emit a startup warning.

### Observability
Publish a lightweight trace topic `perception/decision/trace` with `stamp_ns`, `publish_mode`, `reason`, and `wait_ms` for every published frame.

Expose cumulative counters for:
- `published_fused`
- `published_raw_fallback`
- `no_fused`
- `late_fused`
- `stale_fused`
- `count_mismatch`
- `cache_evicted`
- `duplicate_after_finalize`
- `pending_cache_size_max`

The node should log:
- startup summary with mode, holdoff, timing budget, queue size, mismatch warning
- throttled periodic counter summary

### Testing
Add unit tests for:
- late fused after deadline cannot merge
- fused after raw-finalized cannot rewrite
- merge path rejects overdue fused when flush is delayed
- stale/evicted entries increment counters

Also keep a launch/config contract test so the adapter no longer hardcodes a separate default holdoff in `perception.launch`.
