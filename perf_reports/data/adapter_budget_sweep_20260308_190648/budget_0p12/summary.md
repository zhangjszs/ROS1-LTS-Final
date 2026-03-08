# Adapter Replay Summary

| Scenario | Frames | Fused | Raw Fallback | Reason Counts | wait_ms p50/p95/p99/max | output_gap_ms p99/max |
| --- | ---: | ---: | ---: | --- | --- | --- |
| raw_only | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `121.8/122.4/122.4/122.5` | `100.5/100.6` |
| same_stamp_normal_merge | 40 | 40 (100.0%) | 0 (0.0%) | `{"normal_merge": 40}` | `10.2/10.6/10.6/10.7` | `100.9/100.9` |
| late_fused_within_budget | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `120.9/121.3/121.4/121.5` | `100.6/100.7` |
| late_fused_over_deadline | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `128.5/129.2/129.3/129.3` | `100.2/100.3` |
| intermittent_drop_count_mismatch | 48 | 17 (35.4%) | 31 (64.6%) | `{"count_mismatch": 12, "late_fused": 7, "no_fused": 12, "normal_merge": 17}` | `70.1/121.6/121.8/121.8` | `200.4/200.4` |
| duplicate_stale_finalize | 36 | 18 (50.0%) | 18 (50.0%) | `{"no_fused": 18, "normal_merge": 18}` | `72.6/125.5/125.6/125.6` | `200.3/200.3` |
| callback_timer_pressure | 80 | 0 (0.0%) | 80 (100.0%) | `{"no_fused": 80}` | `129.1/129.5/129.6/129.6` | `50.4/50.5` |
