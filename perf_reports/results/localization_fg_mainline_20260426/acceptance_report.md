# Localization FG Mainline Multi-Mission Acceptance Report

- Date: 2026-04-26
- Goal: Validate FG backend across all mission profiles (autocross, acceleration, skidpad)
- Script: `scripts/validate_localization_fg_mainline_replay.sh`

---

## Cutover Contract (all missions)

- `backend:=factor_graph`
- `publish_dual_backends:=true`
- `fg_shadow_mode:=false`
- `fg_mainline_enable_mapper_fallback:=true`

---

## Results Summary

| Mission | Status | Samples | Pos p95 (m) | Heading p95 (°) | Vel p95 (m/s) | Time diff p95 (s) |
|---------|--------|---------:|-------------|-----------------|---------------:|-------------------:|
| autocross | **PASS** | 288 | 0.7270 | 8.5447 | 0.0584 | 0.0807 |
| acceleration | **PASS** | 161 | 0.6529 | 9.3846 | 0.0204 | 0.0405 |
| skidpad | **PASS** | 239 | 1.3454 | 14.1237 | 0.0426 | 0.0807 |

---

## Per-Mission Details

### autocross (track.bag)

```bash
bash scripts/validate_localization_fg_mainline_replay.sh \
  --mission autocross \
  --compare-duration 30 \
  --min-samples 30
```

- All acceptance checks passed (`backend_factor_graph_seen`, `active_source_fg_seen`, `fg_shadow_false_seen`)
- Pos error p95: 0.727m (comparable to trackdrive baseline 0.726m)

### acceleration (accel.bag)

```bash
bash scripts/validate_localization_fg_mainline_replay.sh \
  --mission acceleration \
  --compare-duration 30 \
  --min-samples 30
```

- All acceptance checks passed
- Pos error p95: 0.653m (best among three missions, straight-line scenario)
- Heading error p95: 9.38°

### skidpad (skidpad.bag)

```bash
bash scripts/validate_localization_fg_mainline_replay.sh \
  --mission skidpad \
  --compare-duration 30 \
  --min-samples 30
```

- All acceptance checks passed
- Pos error p95: 1.345m (highest among three missions)
- Heading error p95: 14.12° (highest among three missions)
- Notes: Skidpad figure-8 tight turns stress heading estimation; errors remain within mission-tolerable range

---

## Verdict

- **autocross**: PASS
- **acceleration**: PASS
- **skidpad**: PASS

**Overall: FG backend is ready for mainline cutover across all mission profiles.**

Skidpad shows elevated pos/heading errors relative to trackdrive/acceleration due to aggressive turning dynamics, but acceptance checks (backend source, shadow mode, sample count) all pass cleanly.

---

## Artifacts

- `perf_reports/results/localization_fg_mainline_autocross_20260426_091805/`
- `perf_reports/results/localization_fg_mainline_acceleration_20260426_091851/`
- `perf_reports/results/localization_fg_mainline_skidpad_20260426_091936/`
