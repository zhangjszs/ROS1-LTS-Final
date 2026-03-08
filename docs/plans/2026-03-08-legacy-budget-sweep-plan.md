# Legacy Budget Sweep Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Measure whether a single tighter `vision_inject.max_age_sec` default can reduce wait cost without breaking the hardened legacy adapter semantics.

**Architecture:** Reuse the existing adapter-only replay harness unchanged and sweep only the legacy timing budget via `LEGACY_BUDGET_SEC`. For each budget, run the same 7 synthetic 1x scenarios, keep the same trace-based metrics, then compare all budgets against the existing `0.2s` baseline using one summary table.

**Tech Stack:** ROS Noetic, `roslaunch`, `rosbag`, Bash orchestration, Python offline JSON aggregation.

---

### Task 1: Freeze scope

**Files:**
- Reference: `scripts/run_adapter_replay_suite.sh`
- Reference: `scripts/analyze_adapter_replay.py`
- Reference: `src/perception_ros/launch/cone_detection_adapter_only.launch`

**Step 1: Confirm parameter surface**

Check that `LEGACY_BUDGET_SEC` already feeds `legacy_budget_sec`, which sets `/perception/lidar_cluster/lidar_cluster_node/vision_inject/max_age_sec`.

**Step 2: Confirm no logic changes are required**

Reuse the existing 7-scenario publisher, trace topic, and offline analyzer. Do not modify adapter state-machine code or replay semantics.

### Task 2: Run the sweep

**Files:**
- Output: `perf_reports/data/adapter_replay_budget_0p08_<timestamp>/`
- Output: `perf_reports/data/adapter_replay_budget_0p10_<timestamp>/`
- Output: `perf_reports/data/adapter_replay_budget_0p12_<timestamp>/`
- Output: `perf_reports/data/adapter_replay_budget_0p15_<timestamp>/`

**Step 1: Execute 1x replay for each candidate budget**

Run the existing suite with `LEGACY_BUDGET_SEC` set to `0.08`, `0.10`, `0.12`, and `0.15`.

**Step 2: Preserve the existing `0.2s` baseline**

Use the already-recorded baseline at `perf_reports/data/adapter_replay_20260308_185454/summary.json` as the `0.2s` comparison point instead of rerunning it.

### Task 3: Aggregate cross-budget evidence

**Files:**
- Output: `perf_reports/data/adapter_budget_sweep_<timestamp>/comparison.md`
- Output: `perf_reports/data/adapter_budget_sweep_<timestamp>/comparison.json`

**Step 1: Read per-budget `summary.json` artifacts**

Extract per-scenario metrics:
- `frames`
- `fused/raw_fallback` counts and ratios
- `reason_counts`
- `wait_ms p50/p95/p99/max`
- `output_gap_ms p99/max`

**Step 2: Build one comparison table**

For each scenario, place the `0.2s` baseline alongside `0.08`, `0.10`, `0.12`, and `0.15` so budget tradeoffs are directly visible.

### Task 4: Make the default-budget recommendation

**Files:**
- Reference: sweep comparison artifacts above

**Step 1: Check correctness non-regression**

Reject any budget that breaks the hardened semantics:
- no post-deadline rewrite
- no post-finalize rewrite
- stale/duplicate only counted, not published

**Step 2: Compare delay vs. fusion tradeoff**

Use these primary decision points:
- `raw_only` wait tail
- `late_fused_within_budget` fused ratio and wait tail
- `callback_timer_pressure` wait tail
- mixed-scenario `output_gap_ms p99/max`

**Step 3: Choose next action**

Either:
- select a new single global default, or
- conclude that no single budget is acceptable and defer to a later mission-specific split
