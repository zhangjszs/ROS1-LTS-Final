# Adapter Replay Evidence Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Produce 1x replay evidence for seven adapter timing scenarios, including per-scenario result bags and a summary table of frame mix, reason buckets, wait distribution, and output gap distribution.

**Architecture:** Run the adapter in isolation under a minimal launch with legacy timing params, drive synthetic raw/fused sequences from a scenario publisher at wall-clock rate, record `perception/decision/detections` and `perception/decision/trace`, then analyze each bag offline into CSV/Markdown summary artifacts.

**Tech Stack:** ROS Noetic, rospy, rosbag, bash orchestration, Python 3 offline analysis.

---

### Task 1: Add a minimal adapter-only launch and scenario publisher
- Create a standalone adapter launch with legacy params and canonical topics.
- Create a scenario publisher that emits deterministic raw/fused sequences for the seven replay cases.

### Task 2: Add replay orchestration
- Create a wrapper script that runs each scenario at 1x, records a result bag, and stores logs/artifacts under a dedicated output directory.

### Task 3: Add offline analysis
- Create a script that reads `perception/decision/detections` and `perception/decision/trace` from each bag and computes:
  - frame count
  - fused/raw_fallback ratio
  - reason bucket counts
  - wait_ms p50/p95/p99/max
  - output gap p99/max
- Emit per-scenario JSON/CSV and a combined Markdown table.

### Task 4: Run the seven scenarios and collect artifacts
- Execute the replay wrapper at 1x for all scenarios.
- Verify bags and summary files are produced for each case.

### Task 5: Summarize results
- Present the combined table and call out whether the default legacy budget appears acceptable or should be tightened.
