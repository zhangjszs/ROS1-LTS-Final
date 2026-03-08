# Adapter Replay Summary

| Scenario | Frames | Fused | Raw Fallback | Reason Counts | wait_ms p50/p95/p99/max | output_gap_ms p99/max |
| --- | ---: | ---: | ---: | --- | --- | --- |
| trackdrive_decision | 173 | 0 (0.0%) | 173 (100.0%) | `{"no_fused": 173}` | `125.6/129.6/130.7/132.6` | `111.5/113.1` |
| autocross_decision | 172 | 0 (0.0%) | 172 (100.0%) | `{"no_fused": 172}` | `125.1/129.9/131.1/132.0` | `131.4/141.5` |
| skidpad_decision | 173 | 0 (0.0%) | 173 (100.0%) | `{"no_fused": 173}` | `155.4/159.7/160.7/182.7` | `121.4/122.0` |
| acceleration_decision | 173 | 0 (0.0%) | 173 (100.0%) | `{"no_fused": 173}` | `104.8/109.6/111.0/115.6` | `112.1/121.2` |
