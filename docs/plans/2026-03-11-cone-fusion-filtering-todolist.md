# Cone Fusion Filtering Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the current "LiDAR geometry + vision color overwrite" pipeline with a cone-oriented fusion pipeline that can use YOLO to suppress LiDAR false positives inside the camera FOV.

**Architecture:** Keep LiDAR clustering as the proposal generator, keep YOLO as the 2D cone verifier, and add a candidate-level fusion stage that outputs `vision_support_score`, `association_reason`, and `reject/keep` decisions. Start with a calibrated, rule-based CLOCs-style post-fusion gate, then leave a clean upgrade path to an object-centric learned scorer.

**Tech Stack:** ROS Noetic, C++17, existing `perception_ros` pipeline, `autodrive_msgs`, rosbag replay, GTest, Python replay/evaluation scripts

---

## Context

- Current fusion is not filtering LiDAR false positives. It mainly copies `fused_color_types` back into raw detections.
- Current runtime still uses legacy `vision_inject`, not the full `fusion/*` calibrated path.
- `trackdrive.launch` defaults fusion budget to `0.12s`, which is tight enough to cause `no_fused` and `late_fused`.
- The target task is not "better color injection". The target task is "LiDAR proposal confirmation and false-positive suppression".

## Success Criteria

- Camera-FOV false positive rate drops significantly on replay bags with roadside clutter.
- `perception/decision/detections` removes or down-weights LiDAR-only clutter when YOLO is fresh and image quality is acceptable.
- Fusion decisions are explainable from logs and message fields.
- The system degrades safely to LiDAR-only behavior when image quality is poor, the frame is stale, or the target lies outside the camera FOV.

## Recommended Technical Direction

- Primary implementation target: CLOCs / Fast-CLOCs style candidate-level post-fusion.
- Future upgrade target: object-centric fusion similar to ObjectFusion or SparseFusion.
- Explicitly not the first step: BEVFusion / TransFusion full detector replacement.

## Execution Status (Updated: 2026-03-11)

- [x] Task 1: Freeze baseline (`perf_reports/data/cone_fusion_baseline/README.md`)
- [x] Task 2: Add evaluation labels/metrics (`scripts/eval_cone_fusion_filtering.py`, `metric_schema.md`)
- [x] Task 3: Move off legacy as main path (fusion mainline + launch contracts)
- [x] Task 4: Make time sync first-class (`budget_sweep` + `budget_selection_report.md`)
- [x] Task 5: Add candidate-level fusion features (`HUAT_FusedConeDetections.msg` extension)
- [x] Task 6: Replace color-injection objective with rule-based filter (`fusion_filter_track.yaml`)
- [x] Task 7: Improve association quality + reason contract (`cone_fusion_association_notes.md`)
- [x] Task 8: Add replay cases + regression script (`run_cone_fusion_filter_replay.sh`)
- [x] Task 9 (Optional): Learned scorer path (`cone_fusion_scorer.*`, `train_cone_fusion_scorer.py`)
- [x] Task 10: Final validation + rollout docs (`perf_reports/data/cone_fusion_rollout_report.md`, `docs/modules/perception/lidar_pipeline.md`)

Validation evidence for Task 10:

- `perf_reports/data/cone_fusion_rollout/mission_eval/*_rollout_eval.{json,csv,md}`
- `perf_reports/data/cone_fusion_baseline/filter_replay_20260311_144153/mode_comparison.md`
- `perf_reports/data/cone_fusion_baseline/budget_sweep/budget_selection_report.md`

---

### Task 1: Freeze The Current Baseline

**Files:**
- Read: `/home/kerwin/2025huat/src/perception_ros/src/lidar_cluster_ros.cpp`
- Read: `/home/kerwin/2025huat/src/perception_ros/src/cone_detection_adapter.cpp`
- Read: `/home/kerwin/2025huat/src/perception_ros/config/lidar_base.yaml`
- Read: `/home/kerwin/2025huat/src/fsd_launch/launch/trackdrive.launch`
- Create: `/home/kerwin/2025huat/perf_reports/data/cone_fusion_baseline/README.md`

**Todo:**
- [ ] Record the current fusion behavior as "color injection, not proposal filtering".
- [ ] Export one clutter-heavy replay bag and one clean replay bag for repeated evaluation.
- [ ] Save baseline metrics: raw LiDAR cone count, final decision cone count, `no_fused`, `late_fused`, and planner-visible false positives.
- [ ] Save representative log excerpts proving the current runtime path and timing budget.

**Acceptance:**
- A baseline folder exists with bag references, logs, and a short metric summary.

---

### Task 2: Add Fusion Evaluation Labels And Metrics

**Files:**
- Create: `/home/kerwin/2025huat/scripts/eval_cone_fusion_filtering.py`
- Create: `/home/kerwin/2025huat/perf_reports/data/cone_fusion_baseline/metric_schema.md`
- Modify: `/home/kerwin/2025huat/docs/perception_regression_assets.md`

**Todo:**
- [ ] Define per-frame metrics: raw proposals, fused proposals, rejected proposals, matched proposals, unmatched proposals.
- [ ] Define task metrics: camera-FOV false positive rate, false reject rate, average fusion latency, unmatched ratio, late-fused ratio.
- [ ] Add a replay script that reads rosbag outputs and summarizes these metrics into Markdown or CSV.
- [ ] Separate metrics by scene type: clean cones, clutter, partial occlusion, edge-of-image, low light.

**Acceptance:**
- Running the evaluation script on a replay bag produces a stable metrics report that can compare before and after changes.

---

### Task 3: Move Off Legacy `vision_inject`

**Files:**
- Modify: `/home/kerwin/2025huat/src/perception_ros/config/lidar_base.yaml`
- Modify: `/home/kerwin/2025huat/src/perception_ros/launch/lidar_cluster.launch`
- Modify: `/home/kerwin/2025huat/src/fsd_launch/launch/subsystems/perception.launch`
- Test: `/home/kerwin/2025huat/src/perception_ros/test/test_mainline_adapter_launch_contract.py`

**Todo:**
- [ ] Introduce explicit `fusion/enabled`, `fusion/topics/*`, and `fusion/projection/*` parameters in the base config.
- [ ] Stop relying on legacy-only defaults for `camera_info` and `TF`.
- [ ] Keep a temporary compatibility switch for rollback during replay validation.
- [ ] Update launch contract tests so the mainline path is the calibrated fusion path.

**Acceptance:**
- Startup logs no longer report legacy `vision_inject` as the main intended fusion mode.

---

### Task 4: Make Time Sync A First-Class Constraint

**Files:**
- Modify: `/home/kerwin/2025huat/src/fsd_launch/launch/trackdrive.launch`
- Modify: `/home/kerwin/2025huat/src/perception_ros/nodes/cone_detection_adapter_node.cpp`
- Modify: `/home/kerwin/2025huat/src/perception_ros/src/cone_detection_adapter.cpp`
- Test: `/home/kerwin/2025huat/src/perception_ros/test/test_cone_detection_adapter.cpp`

**Todo:**
- [ ] Expose mission-specific fusion timing budget separately from adapter raw holdoff.
- [ ] Replay-sweep `0.12`, `0.15`, `0.18`, and `0.20` second budgets.
- [ ] Add tests for `late_fused`, `no_fused`, and duplicate-after-finalize behavior under the chosen budget.
- [ ] Choose one default budget for trackdrive based on replay evidence, not intuition.

**Acceptance:**
- A replay report shows which budget minimizes false fallback without creating unacceptable latency.

---

### Task 5: Add Candidate-Level Fusion Features

**Files:**
- Modify: `/home/kerwin/2025huat/src/autodrive_msgs/msg/HUAT_FusedConeDetections.msg`
- Modify: `/home/kerwin/2025huat/src/perception_ros/src/lidar_cluster_ros.cpp`
- Modify: `/home/kerwin/2025huat/src/perception_ros/include/perception_ros/lidar_cluster_ros.hpp`
- Test: `/home/kerwin/2025huat/src/autodrive_msgs/CMakeLists.txt`

**Todo:**
- [ ] Extend fused output with features needed for proposal confirmation:
- [ ] Add projected center distance or normalized pixel residual.
- [ ] Add bbox containment or IoU-like score.
- [ ] Add in-camera-FOV flag.
- [ ] Add image-quality-at-decision and frame-staleness indicators.
- [ ] Add `vision_support_score` and `filter_recommendation` fields.
- [ ] Preserve `association_reason` so every reject/keep decision remains debuggable.

**Acceptance:**
- One fused message is sufficient to explain why a LiDAR candidate was kept, downgraded, or rejected.

---

### Task 6: Replace Color Injection Logic With A Rule-Based Filter

**Files:**
- Modify: `/home/kerwin/2025huat/src/perception_ros/src/lidar_cluster_ros.cpp`
- Modify: `/home/kerwin/2025huat/src/perception_ros/src/cone_detection_adapter.cpp`
- Create: `/home/kerwin/2025huat/src/perception_ros/config/fusion_filter_track.yaml`
- Test: `/home/kerwin/2025huat/src/perception_ros/test/test_cone_detection_adapter.cpp`

**Todo:**
- [ ] Implement a rule-based gate for each LiDAR cone proposal.
- [ ] Rule 1: if the target is inside camera FOV, image quality is acceptable, and YOLO is fresh, require visual support to keep high confidence.
- [ ] Rule 2: if the target is outside camera FOV, do not reject based on missing YOLO support.
- [ ] Rule 3: if image quality is degraded or frame is stale, downgrade confidence instead of hard rejection.
- [ ] Rule 4: if projection is valid but there is no matching box, mark as likely clutter and suppress or demote.
- [ ] Keep color fusion only as a secondary output, not the main objective.

**Acceptance:**
- Final `perception/decision/detections` behavior changes from "always preserve LiDAR candidates" to "filter or down-weight unsupported candidates inside camera coverage".

---

### Task 7: Improve Association Quality For Small Cone Targets

**Files:**
- Modify: `/home/kerwin/2025huat/src/perception_ros/src/lidar_cluster_ros.cpp`
- Modify: `/home/kerwin/2025huat/src/perception_ros/config/lidar_base.yaml`
- Create: `/home/kerwin/2025huat/docs/modules/perception/cone_fusion_association_notes.md`

**Todo:**
- [ ] Stop relying on fixed pixel-distance-only matching for small cones.
- [ ] Use a combined score from projected center error, bbox containment, box scale consistency, and YOLO confidence.
- [ ] Make the pixel threshold distance-aware or box-size-aware.
- [ ] Record failure reasons separately: `camera_info_missing`, `tf_lookup_failed`, `out_of_image`, `no_bbox_match_after_projection`, `low_vision_confidence`.

**Acceptance:**
- Replay diagnostics can distinguish calibration errors from association-threshold errors from genuine visual misses.

---

### Task 8: Add Replay Cases And Regression Tests

**Files:**
- Create: `/home/kerwin/2025huat/scripts/run_cone_fusion_filter_replay.sh`
- Modify: `/home/kerwin/2025huat/src/perception_ros/test/test_cone_detection_adapter.cpp`
- Modify: `/home/kerwin/2025huat/docs/perception_regression_assets.md`

**Todo:**
- [ ] Add replay cases for clutter near cones, clutter without cones, edge-of-image cones, and temporary YOLO misses.
- [ ] Add unit tests covering keep/reject/downgrade branches.
- [ ] Add one regression run that compares raw-only, current legacy fusion, and new filtering fusion.
- [ ] Save before/after reports in `perf_reports/data`.

**Acceptance:**
- The new filter can be regression-tested without manual RViz inspection.

---

### Task 9: Optional Learned Scorer Upgrade

**Files:**
- Create: `/home/kerwin/2025huat/src/perception_ros/src/cone_fusion_scorer.cpp`
- Create: `/home/kerwin/2025huat/src/perception_ros/include/perception_ros/cone_fusion_scorer.hpp`
- Create: `/home/kerwin/2025huat/scripts/train_cone_fusion_scorer.py`
- Create: `/home/kerwin/2025huat/docs/modules/perception/cone_fusion_training_spec.md`

**Todo:**
- [ ] Collect proposal-level features and labels from replay bags.
- [ ] Train a lightweight classifier such as logistic regression, XGBoost, or a tiny MLP.
- [ ] Replace the hard-coded rule threshold with a learned `vision_support_score`.
- [ ] Keep the rule-based path as a fallback and debugging baseline.

**Acceptance:**
- The learned scorer beats the rule-based gate on clutter-heavy replay without increasing false rejects beyond the agreed threshold.

---

### Task 10: Final Validation And Rollout

**Files:**
- Modify: `/home/kerwin/2025huat/docs/plans/2026-03-11-cone-fusion-filtering-todolist.md`
- Create: `/home/kerwin/2025huat/perf_reports/data/cone_fusion_rollout_report.md`
- Modify: `/home/kerwin/2025huat/docs/modules/perception/lidar_pipeline.md`

**Todo:**
- [ ] Run replay validation on all target missions used for track.
- [ ] Document chosen default parameters and rollback knobs.
- [ ] Summarize measured improvements and remaining failure cases.
- [ ] Update the perception documentation so the team understands the new fusion contract.

**Acceptance:**
- There is a rollout report with final parameters, evidence, and known risks.

---

## Immediate Next Actions

- [ ] Confirm that the near-term target is candidate-level post-fusion filtering, not full detector replacement.
- [ ] Replay-sweep the timing budget and stop using `0.12s` by default unless the data justifies it.
- [ ] Migrate to calibrated `fusion/*` config before tuning matching thresholds.
- [ ] Implement the rule-based keep/reject gate before attempting any learned fusion model.

## Non-Goals For The First Iteration

- [ ] Do not replace YOLO.
- [ ] Do not replace LiDAR clustering.
- [ ] Do not start with BEVFusion, TransFusion, or CMT.
- [ ] Do not optimize cone color semantics before false-positive filtering works.
