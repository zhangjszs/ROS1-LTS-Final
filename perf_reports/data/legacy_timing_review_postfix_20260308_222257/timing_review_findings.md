# Legacy Runtime Timing Review

| Mission | Budget | Vision Hz | Vision header->raw p95 | Vision publish lag p50/p95/p99 | Raw frames with header match <= budget | Fusion pair/used/published | Trace no_fused | Trace wait_ms p50/p95/p99/max |
| --- | ---: | ---: | ---: | --- | ---: | --- | ---: | --- |
| trackdrive | 0.12s | 2.29 | 47.1 ms | `466.2/545.7/628.8` | 99/192 (51.6%) | `60/59/59` | 100.0% | `124.9/129.8/130.9/132.0` |
| autocross | 0.12s | 2.17 | 49.6 ms | `489.4/545.1/566.6` | 90/192 (46.9%) | `59/59/59` | 100.0% | `125.2/129.3/130.4/133.5` |
| skidpad | 0.15s | 2.10 | 48.1 ms | `665.4/809.3/827.5` | 116/192 (60.4%) | `55/55/55` | 100.0% | `155.1/159.6/160.6/161.0` |
