# Adapter Replay Summary

| Scenario | Frames | Fused | Raw Fallback | Reason Counts | wait_ms p50/p95/p99/max | output_gap_ms p99/max |
| --- | ---: | ---: | ---: | --- | --- | --- |
| raw_only | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `152.3/152.8/152.9/152.9` | `100.5/100.5` |
| same_stamp_normal_merge | 40 | 40 (100.0%) | 0 (0.0%) | `{"normal_merge": 40}` | `10.1/10.4/10.5/10.6` | `101.0/101.1` |
| late_fused_within_budget | 40 | 22 (55.0%) | 18 (45.0%) | `{"late_fused": 18, "normal_merge": 22}` | `149.9/150.6/150.8/151.0` | `101.3/101.4` |
| late_fused_over_deadline | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `154.7/155.2/155.3/155.4` | `100.3/100.3` |
| intermittent_drop_count_mismatch | 48 | 24 (50.0%) | 24 (50.0%) | `{"count_mismatch": 12, "no_fused": 12, "normal_merge": 24}` | `70.1/152.3/152.4/152.4` | `200.2/200.4` |
| duplicate_stale_finalize | 36 | 18 (50.0%) | 18 (50.0%) | `{"no_fused": 18, "normal_merge": 18}` | `88.2/157.0/157.0/157.0` | `199.6/199.6` |
| callback_timer_pressure | 80 | 0 (0.0%) | 80 (100.0%) | `{"no_fused": 80}` | `157.5/158.0/158.1/158.2` | `50.5/50.5` |
