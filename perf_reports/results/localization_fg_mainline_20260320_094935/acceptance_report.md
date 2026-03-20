# Localization FG Mainline Migration Acceptance Report

- Date: 2026-03-20
- Mission: `trackdrive`
- Goal: Validate shadow->mainline cutover regression for localization FG backend (no parameter tuning)
- Script: `scripts/validate_localization_fg_mainline_replay.sh`

## Replay Command

```bash
OUT_DIR=/home/kerwin/2025huat/perf_reports/results/localization_fg_mainline_20260320_094935 \
  bash scripts/validate_localization_fg_mainline_replay.sh \
  --mission trackdrive \
  --compare-duration 40 \
  --min-samples 40
```

## Cutover Contract

- `backend:=factor_graph`
- `publish_dual_backends:=true`
- `fg_shadow_mode:=false`
- `fg_mainline_enable_mapper_fallback:=true`

## Acceptance Checks

- `backend_factor_graph_seen`: `yes`
- `active_source_fg_seen`: `yes`
- `fg_shadow_false_seen`: `yes`
- `samples`: `339` (>= `40`)

## Metrics (p95)

- `pos_error_m`: `0.726427`
- `heading_error_deg`: `5.495607`
- `velocity_error_mps`: `0.050401`
- `time_diff_s`: `0.080743`

## Verdict

- **PASS**

## Artifacts

- `perf_reports/results/localization_fg_mainline_20260320_094935/results.tsv`
- `perf_reports/results/localization_fg_mainline_20260320_094935/summary.md`
- `perf_reports/results/localization_fg_mainline_20260320_094935/result.json`
- `perf_reports/results/localization_fg_mainline_20260320_094935/backend_comparison.csv`
- `perf_reports/results/localization_fg_mainline_20260320_094935/backend_comparison.png`
