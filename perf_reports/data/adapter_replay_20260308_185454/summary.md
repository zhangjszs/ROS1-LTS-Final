# Adapter Replay Summary

| Scenario | Frames | Fused | Raw Fallback | Reason Counts | wait_ms p50/p95/p99/max | output_gap_ms p99/max |
| --- | ---: | ---: | ---: | --- | --- | --- |
| raw_only | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `204.7/205.1/205.2/205.3` | `100.3/100.3` |
| same_stamp_normal_merge | 40 | 40 (100.0%) | 0 (0.0%) | `{"normal_merge": 40}` | `10.0/10.4/10.5/10.5` | `100.9/101.0` |
| late_fused_within_budget | 40 | 40 (100.0%) | 0 (0.0%) | `{"normal_merge": 40}` | `149.9/150.5/150.7/150.7` | `101.0/101.1` |
| late_fused_over_deadline | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `201.4/201.8/202.0/202.0` | `100.2/100.3` |
| intermittent_drop_count_mismatch | 48 | 24 (50.0%) | 24 (50.0%) | `{"count_mismatch": 12, "no_fused": 12, "normal_merge": 24}` | `70.2/209.0/209.1/209.2` | `200.6/200.7` |
| duplicate_stale_finalize | 36 | 18 (50.0%) | 18 (50.0%) | `{"no_fused": 18, "normal_merge": 18}` | `110.3/209.8/209.9/210.0` | `199.9/200.0` |
| callback_timer_pressure | 80 | 48 (60.0%) | 32 (40.0%) | `{"count_mismatch": 16, "no_fused": 16, "normal_merge": 48}` | `190.0/203.1/203.6/203.6` | `63.8/63.9` |
