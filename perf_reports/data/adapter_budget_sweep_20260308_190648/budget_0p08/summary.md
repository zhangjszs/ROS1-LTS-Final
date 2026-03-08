# Adapter Replay Summary

| Scenario | Frames | Fused | Raw Fallback | Reason Counts | wait_ms p50/p95/p99/max | output_gap_ms p99/max |
| --- | ---: | ---: | ---: | --- | --- | --- |
| raw_only | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `89.4/89.9/90.0/90.0` | `110.2/110.2` |
| same_stamp_normal_merge | 40 | 40 (100.0%) | 0 (0.0%) | `{"normal_merge": 40}` | `10.1/10.3/10.4/10.5` | `100.8/100.9` |
| late_fused_within_budget | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `85.1/85.5/85.6/85.7` | `100.4/100.5` |
| late_fused_over_deadline | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `89.0/89.3/89.3/89.4` | `100.3/100.3` |
| intermittent_drop_count_mismatch | 48 | 12 (25.0%) | 36 (75.0%) | `{"count_mismatch": 12, "no_fused": 24, "normal_merge": 12}` | `50.4/89.9/90.0/90.0` | `170.5/170.5` |
| duplicate_stale_finalize | 36 | 18 (50.0%) | 18 (50.0%) | `{"no_fused": 18, "normal_merge": 18}` | `50.3/81.2/81.4/81.4` | `161.2/161.3` |
| callback_timer_pressure | 80 | 0 (0.0%) | 80 (100.0%) | `{"no_fused": 80}` | `80.8/90.0/90.0/90.0` | `60.4/60.5` |
