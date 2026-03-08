# Legacy Runtime Timing Review

| Mission | Budget | Vision Hz | Vision header->raw p95 | Vision publish lag p50/p95/p99 | Raw frames with header match <= budget | Fusion pair/used/published | Trace no_fused | Trace wait_ms p50/p95/p99/max |
| --- | ---: | ---: | ---: | --- | ---: | --- | ---: | --- |
| trackdrive | 0.12s | 2.29 | 48.9 ms | `7804.9/7986.4/8013.6` | 57/192 (29.7%) | `1/0/0` | 100.0% | `125.9/129.8/132.4/149.8` |
| autocross | 0.12s | 2.37 | 48.3 ms | `7457.9/7825.4/7853.8` | 66/192 (34.4%) | `1/1/1` | 100.0% | `126.4/130.2/131.4/133.0` |
| skidpad | 0.15s | 2.15 | 49.8 ms | `7615.2/10536.5/10810.9` | 82/191 (42.9%) | `8/8/8` | 100.0% | `155.5/159.7/160.2/161.1` |

## Acceleration

{
  "camera_related_topics": []
}
