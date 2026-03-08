# LiDAR Track Mode (18m+) Tuning Design

**Date:** 2026-02-20  
**Scope:** `perception_ros` LiDAR cluster pipeline parameters (Track mode only)  
**Goal:** Reduce 18m+ false positives from continuous vertical structures (walls/guardrails/front barriers) while recovering far-range cone recall, without changing ground segmentation method.

## Context

- Parameter stack is `Base + Overlay`:
  - Base: `src/perception_ros/config/lidar_base.yaml`
  - Track overlay: `src/perception_ros/config/lidar_track.yaml`
  - Loaded by: `src/perception_ros/launch/lidar_cluster.launch`
- User observations:
  - Issues start mainly after ~18m.
  - Ground segmentation is stable.
  - False positives mostly from continuous vertical structures (left/right and also front).

## Hypothesis

18m+ false positives are dominated by non-cone vertical structures that survive:
1) ROI cropping (including adaptive_y trapezoid), and/or
2) post-ground filtering,
then form small clusters that pass confidence thresholds.

Therefore, tuning should first reduce these structures *before* confidence gating, then adjust clustering sensitivity in the 15–35m band.

## Approach (Recommended: Geometry-First Gate)

### A1. Strengthen obstacle-height filter for far range

In `perception_core::lidar_cluster::PostGroundFilter`, the `filters/obstacle_height/*` filter removes far-range grid cells with large z-span, which is a good proxy for walls/guardrails.

Track overlay will:
- Increase `filters/obstacle_height/max_z_span` (allow larger vertical span before filtering triggers on cones; filter still triggers strongly for tall structures).
- Increase `filters/obstacle_height/min_points_to_judge` to avoid accidentally filtering sparse far cones.

### A2. Tighten clustering tolerance in 15–35m segments

Reduce `mode_presets/track/cluster/cluster_tolerance` specifically for:
- 15–25m
- 25–35m

This reduces the chance that wall fragments aggregate into “cone-like” clusters and improves separation of close objects at mid-far range.

### A3. Earlier ROI lateral narrowing (secondary)

Adjust `mode_presets/track/adaptive_y` to start narrowing earlier and reduce `far_y_half`.

This mainly helps lateral walls; front-facing barriers should be addressed primarily by the obstacle-height filter.

## Non-goals (for this iteration)

- Changing ground method (FGS/RANSAC/Patchwork++).
- Tracker/topology behavior changes (risk of increased jitter).
- Performance optimization (deferred).

## Validation Plan

We will validate on `track.bag` replay:
- Replay: `roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/home/kerwin/rosbag/track.bag rate:=2.0 launch_rviz:=false launch_viz:=false`
- Record: `rosbag record /perception/lidar_cluster/detections -O <temp_output>.bag`
- Compute metrics from recorded bag (proxy metrics):
  - mean/std detections, spike rate, zero-frame rate
  - mean/std confidence, mean distance, symmetry ratio, mean frame interval
- Add a simple “wall FP proxy” report:
  - fraction of detections with `distance >= 18m` and `abs(y) >= 6m` (likely outside track corridor)
- Add a “front-center FP proxy” report:
  - fraction of detections with `distance >= 18m` and `abs(y) <= 1m` (unlikely to be true cones in track mode)

Acceptance for this iteration:
- “Wall FP proxy” decreases noticeably.
- “Front-center FP proxy” decreases noticeably.
- Zero-frame rate does not increase materially.
- Mean confidence and stability metrics do not regress severely.
