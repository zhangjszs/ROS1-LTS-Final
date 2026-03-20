# Localization FG Mainline Validation

| mission | status | samples | backend_factor_graph_seen | active_source_fg_seen | fg_shadow_false_seen | pos_p95_m | heading_p95_deg | vel_p95_mps | time_diff_p95_s | notes |
|---|---|---:|---|---|---|---:|---:|---:|---:|---|
| trackdrive | FAIL | 359 | yes | no | no | 1.7592 | 76.7270 | 0.0405 | 0.0000 | active_backend_source!=factor_graph in diagnostics; fg_shadow_mode!=false in diagnostics |
