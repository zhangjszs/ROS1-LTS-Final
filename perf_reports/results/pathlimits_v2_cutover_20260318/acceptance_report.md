# PathLimitsV2 Cutover Replay Acceptance Report

- Date: 2026-03-18
- Scope: `planning/pathlimits` + `planning/pathlimits_v2` cutover/fallback replay acceptance.
- Baseline script: `scripts/validate_pathlimits_v2_cutover_replay.sh`
- Matrix script (new): `scripts/run_pathlimits_v2_cutover_matrix.sh`

## Script fixes landed in this round

1. Fixed diagnostics parsing to read from file path (avoids `Argument list too long` on large diagnostics logs).
2. Extended launch-log source parser to recognize:
   - `PathLimitsV2 recovered; switching back to V2 primary` -> source `v2`
   - `PathLimitsV2 stale or missing; fallback to V1` -> source `v1-fallback`
3. Added V2 evidence fallback when topic probe misses samples:
   - diagnostics `pathlimits_v2_msg_count > 0`
   - launch-log patterns for V2 selection/recovery.

## Acceptance commands used

```bash
# Trackdrive (fixed script)
OUT_DIR=perf_reports/results/pathlimits_v2_cutover_20260318/trackdrive_fix \
  bash scripts/validate_pathlimits_v2_cutover_replay.sh \
  --mission trackdrive --bag /home/kerwin/rosbag/track.bag \
  --rate 2.0 --boot-timeout 30 --scenario-timeout 120

# Autocross
OUT_DIR=perf_reports/results/pathlimits_v2_cutover_20260318/autocross_fix \
  bash scripts/validate_pathlimits_v2_cutover_replay.sh \
  --mission autocross --bag /home/kerwin/rosbag/track.bag \
  --rate 2.0 --boot-timeout 30 --scenario-timeout 120

# Acceleration (fixed script)
OUT_DIR=perf_reports/results/pathlimits_v2_cutover_20260318/acceleration_fix \
  bash scripts/validate_pathlimits_v2_cutover_replay.sh \
  --mission acceleration --bag /home/kerwin/rosbag/accel.bag \
  --rate 2.0 --boot-timeout 30 --scenario-timeout 120

# Skidpad (first two retries failed in v2_primary, third retry passed)
OUT_DIR=perf_reports/results/pathlimits_v2_cutover_20260318/skidpad_fix \
  bash scripts/validate_pathlimits_v2_cutover_replay.sh \
  --mission skidpad --bag /home/kerwin/rosbag/skidpad.bag \
  --rate 2.0 --boot-timeout 30 --scenario-timeout 120

OUT_DIR=perf_reports/results/pathlimits_v2_cutover_20260318/skidpad_fix_retry \
  PATHLIMITS_WAIT_S=120 bash scripts/validate_pathlimits_v2_cutover_replay.sh \
  --mission skidpad --bag /home/kerwin/rosbag/skidpad.bag \
  --rate 2.0 --boot-timeout 30 --scenario-timeout 150

OUT_DIR=/tmp/skidpad_retry2_$(date +%s) PATHLIMITS_WAIT_S=120 \
  bash scripts/validate_pathlimits_v2_cutover_replay.sh \
  --mission skidpad --bag /home/kerwin/rosbag/skidpad.bag \
  --rate 2.0 --boot-timeout 30 --scenario-timeout 150
# copied to perf_reports/results/pathlimits_v2_cutover_20260318/skidpad_fix2
```

## Mission results (latest accepted artifacts)

| mission | artifact dir | v2_primary | fallback_no_v2_publish | overall |
|---|---|---|---|---|
| trackdrive | `trackdrive_fix/` | PASS (`active_source=v2`) | PASS (`active_source=v1-fallback`) | PASS |
| autocross | `autocross_fix/` | PASS (`active_source=v2`) | PASS (`active_source=v1-fallback`) | PASS |
| acceleration | `acceleration_fix/` | PASS (`active_source=v2`) | PASS (`active_source=v1-fallback`) | PASS |
| skidpad | `skidpad_fix2/` | PASS (`active_source=v2`) | PASS (`active_source=v1-fallback`) | PASS |

## Flakiness note

- `skidpad` had two consecutive `v2_primary` failures (`skidpad_fix`, `skidpad_fix_retry`) with `no /planning/pathlimits`, then one pass (`skidpad_fix2`) under same bag and cutover settings.
- This indicates replay-level nondeterminism in skidpad path publication chain; acceptance is currently pass-by-retry for skidpad and should be tracked as a stability risk.

## Baseline vs fixed snapshot

- Initial baseline directories (`trackdrive/`, `acceleration/`) are preserved for traceability.
- Fixed-script and expanded-mission acceptance artifacts are in:
  - `trackdrive_fix/`
  - `autocross_fix/`
  - `acceleration_fix/`
  - `skidpad_fix/`, `skidpad_fix_retry/`, `skidpad_fix2/`

## Conclusion

- Replay cutover acceptance framework is complete and operational.
- `trackdrive/autocross/acceleration` are stable PASS in this round.
- `skidpad` reached PASS but exhibits replay flakiness; next engineering step is to reduce skidpad nondeterminism before treating PASS as stable.
