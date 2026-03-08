# Adapter Replay Summary

| Scenario | Frames | Fused | Raw Fallback | Reason Counts | wait_ms p50/p95/p99/max | output_gap_ms p99/max |
| --- | ---: | ---: | ---: | --- | --- | --- |
| raw_only | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `104.9/105.3/105.5/105.5` | `100.6/100.7` |
| same_stamp_normal_merge | 40 | 40 (100.0%) | 0 (0.0%) | `{"normal_merge": 40}` | `10.1/10.5/10.7/10.7` | `100.8/100.9` |
| late_fused_within_budget | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `105.6/106.2/106.3/106.3` | `100.3/100.3` |
| late_fused_over_deadline | 40 | 0 (0.0%) | 40 (100.0%) | `{"no_fused": 40}` | `104.2/104.8/105.0/105.0` | `100.3/100.3` |
| intermittent_drop_count_mismatch | 48 | 12 (25.0%) | 36 (75.0%) | `{"count_mismatch": 12, "no_fused": 24, "normal_merge": 12}` | `61.8/104.0/104.1/104.1` | `184.2/184.3` |
| duplicate_stale_finalize | 36 | 18 (50.0%) | 18 (50.0%) | `{"no_fused": 18, "normal_merge": 18}` | `64.1/108.6/108.7/108.8` | `189.0/189.0` |
| callback_timer_pressure | 80 | 0 (0.0%) | 80 (100.0%) | `{"no_fused": 80}` | `106.7/107.2/107.3/107.3` | `50.7/50.8` |
