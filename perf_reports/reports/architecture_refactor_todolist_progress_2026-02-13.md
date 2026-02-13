# Architecture Refactor Progress (2026-02-13)

This report follows the original low-coupling/high-cohesion TODO sequence and records implemented changes plus verification evidence.

## Original TODO and Status

1. Fix `full_simulation.launch` planner/mission semantics (P0)
- Status: Done
- Evidence:
  - `src/fsd_launch/launch/simulation/full_simulation.launch:5`
  - `src/fsd_launch/launch/simulation/full_simulation.launch:6`
  - `src/fsd_launch/launch/simulation/full_simulation.launch:8`
  - `src/fsd_launch/launch/simulation/full_simulation.launch:9`
  - `src/fsd_launch/launch/simulation/full_simulation.launch:50`
  - `src/fsd_launch/launch/simulation/full_simulation.launch:51`
- Result:
  - Canonical semantics are now `planner=unified|none`, `mission=...`.
  - Legacy usage `planner:=high_speed|line|skidpad|...` is backward-compatible via resolver args.

2. Add `cmd_topic` passthrough for vehicle subsystem (P0)
- Status: Done
- Evidence:
  - `src/fsd_launch/launch/subsystems/vehicle.launch:6`
  - `src/fsd_launch/launch/subsystems/vehicle.launch:10`
  - `src/vehicle_interface_ros/launch/vehicle_interface.launch:3`
  - `src/vehicle_interface_ros/launch/vehicle_interface.launch:7`
- Result:
  - Command topic is now configurable from orchestration layer instead of hidden default coupling.

3. Clean legacy topic comments in control config + VCU mapping doc (P0)
- Status: Done
- Evidence:
  - `src/control_ros/config/param.yaml:10`
  - `src/control_ros/config/param.yaml:182`
  - `src/control_ros/config/control_accel.yaml:82`
  - `src/control_ros/config/control_track.yaml:72`
  - `src/control_ros/config/control_skidpad.yaml:86`
  - `src/fsd_launch/reference/vcu_mapping.md:25`
  - `src/fsd_launch/reference/vcu_mapping.md:33`
- Result:
  - Canonical topics documented as `vehicle/cmd`, `localization/car_state`, `planning/pathlimits`.
  - Legacy topics kept as optional compatibility notes only.

4. Non-simulation chain static consistency validation (follow-up from original TODO)
- Status: Done
- Method:
  - `roslaunch --dump-params` for `trackdrive/autocross/skidpad/acceleration/ebs_test`.
  - Extracted key params for `localization/planning/control/vehicle` contract.
- Results (with injected values `verify/*`):
  - `trackdrive/autocross/skidpad/acceleration` all propagate:
    - `/location_node/topics/car_state_in|out = verify/state`
    - `/planning_pipeline/input_pose_topic = verify/state`
    - `/planning_pipeline/output_pathlimits_topic = verify/path`
    - `/planning_pipeline/output_approaching_goal_topic = verify/goal`
    - `/control/carstate_topic = verify/state`
    - `/control/pathlimits_topic = verify/path`
    - `/control/approaching_goal_topic = verify/goal`
    - `/control/cmd_topic = verify/cmd`
    - `/vehicle_interface/cmd_topic = verify/cmd`
  - `ebs_test(enable_ebs_control:=true)` correctly propagates control/vehicle/localization; planning remains disabled by design (`planner:=none`).

## Additional Convergence Completed While Following the Same Refactor Direction

A. Mission-level single-source contract for main chain topics
- Added mission-level args and passthrough for:
  - `cmd_topic`, `carstate_topic`, `pathlimits_topic`, `approaching_goal_topic`
- Evidence:
  - `src/fsd_launch/launch/trackdrive.launch:25`
  - `src/fsd_launch/launch/autocross.launch:24`
  - `src/fsd_launch/launch/skidpad.launch:25`
  - `src/fsd_launch/launch/acceleration.launch:25`
  - `src/fsd_launch/launch/ebs_test.launch:31`
  - `src/fsd_launch/launch/simulation/full_simulation.launch:11`

B. Planning contract wiring hardened (launch + node)
- Evidence:
  - `src/planning_ros/launch/planning_pipeline.launch:7`
  - `src/planning_ros/launch/planning_pipeline.launch:21`
  - `src/planning_ros/launch/planning_pipeline.launch:22`
  - `src/planning_ros/launch/planning_pipeline.launch:23`
  - `src/planning_ros/launch/planning_pipeline.launch:24`
  - `src/planning_ros/src/planning_pipeline_node.cpp:70`
  - `src/planning_ros/src/planning_pipeline_node.cpp:87`
  - `src/planning_ros/src/planning_pipeline_node.cpp:100`

C. Localization output topic made orchestratable from subsystem/mission
- Evidence:
  - `src/localization_ros/launch/location.launch:4`
  - `src/localization_ros/launch/location.launch:17`
  - `src/localization_ros/launch/location_eskf.launch:4`
  - `src/localization_ros/launch/location_eskf.launch:8`
  - `src/fsd_launch/launch/subsystems/localization.launch:8`
  - `src/fsd_launch/launch/subsystems/localization.launch:28`

D. Simulation launch canonicalized to relative topic names
- Evidence:
  - `src/simulation_ros/launch/simulation.launch:5`
  - `src/simulation_ros/launch/simulation.launch:6`
- Result:
  - Better namespace friendliness, reduced hidden absolute-topic coupling.

E. Removed runtime parameter side-effects in planning pipeline initialization
- Evidence:
  - `src/planning_ros/src/planning_pipeline_node.cpp:66`
  - `src/planning_ros/src/planning_pipeline_node.cpp:80`
  - `src/planning_ros/src/planning_pipeline_node.cpp:101`
  - `src/planning_ros/launch/planning_pipeline.launch:21`
  - `src/planning_ros/launch/planning_pipeline.launch:22`
  - `src/planning_ros/launch/planning_pipeline.launch:23`
  - `src/planning_ros/launch/planning_pipeline.launch:24`
- Result:
  - Contract topics are now declared by launch and consumed by nodes, instead of being implicitly mutated by runtime `setParam`.

F. Perception chain joined the same `carstate_topic` contract
- Evidence:
  - `src/fsd_launch/launch/subsystems/perception.launch:7`
  - `src/perception_ros/launch/lidar_cluster.launch:7`
  - `src/perception_ros/launch/lidar_cluster.launch:26`
  - `src/fsd_launch/launch/trackdrive.launch:53`
  - `src/fsd_launch/launch/autocross.launch:34`
  - `src/fsd_launch/launch/skidpad.launch:34`
  - `src/fsd_launch/launch/acceleration.launch:34`
  - `src/fsd_launch/launch/ebs_test.launch:42`
  - `src/fsd_launch/launch/simulation/full_simulation.launch:41`
- Result:
  - Mission-level `carstate_topic` now propagates to perception/localization/planning/control consistently.

G. Compatibility toggles promoted to mission layer (explicit, default-off)
- Evidence:
  - `src/fsd_launch/launch/trackdrive.launch:29`
  - `src/fsd_launch/launch/autocross.launch:28`
  - `src/fsd_launch/launch/skidpad.launch:28`
  - `src/fsd_launch/launch/acceleration.launch:28`
  - `src/fsd_launch/launch/ebs_test.launch:35`
  - `src/fsd_launch/launch/simulation/full_simulation.launch:15`
- Result:
  - Legacy bridge behavior is now a visible orchestration decision (`publish_legacy_cmd_topic`, `subscribe_legacy_topics`, `subscribe_legacy_cmd_topic`, and legacy topic names), no longer hidden inside subsystem defaults.

H. Added automated contract checker script (mission-wide)
- Evidence:
  - `scripts/check_topic_contracts.sh`
- Scope:
  - Verifies standard missions: `trackdrive`, `autocross`, `skidpad`, `acceleration`
  - Verifies `ebs_test(enable_ebs_control:=true)`
  - Verifies `full_simulation` for both `planner:=unified` and `planner:=none`
  - Verifies legacy compatibility toggles propagation path
- Result:
  - Current run result: `PASSED`

I. Wired contract checker into CI workflow
- Evidence:
  - `.github/workflows/architecture-contract.yml`
- Result:
  - CI now includes a dedicated architecture-contract gate that builds required packages and runs `scripts/check_topic_contracts.sh`.

J. Added runtime smoke check script and CI stage
- Evidence:
  - `scripts/check_runtime_smoke.sh`
  - `.github/workflows/architecture-contract.yml` (step: `Run runtime smoke checks`)
- Result:
  - Runtime smoke gate is now part of CI pipeline design.
  - Script now supports strict/non-strict behavior:
    - non-strict (default local): permission-blocked env => `SKIPPED`
    - strict (`RUNTIME_SMOKE_REQUIRED=1`): permission-blocked env => `FAILED`
  - Local execution in this sandbox still hits `netifaces` permission limits in `roslaunch` machine-local detection, but behavior is now explicit and controlled by mode.

K. Centralized topic/frame magic strings into shared contract header
- Evidence:
  - `src/autodrive_msgs/include/autodrive_msgs/topic_contract.hpp`
  - `src/autodrive_msgs/CMakeLists.txt`
  - `src/control_ros/src/control_node.cpp`
  - `src/planning_ros/src/planning_pipeline_node.cpp`
  - `src/planning_ros/src/line_detection_node.cpp`
  - `src/planning_ros/src/skidpad_detection_node.cpp`
  - `src/planning_ros/src/high_speed_tracking/utils/Params.cpp`
  - `src/planning_ros/src/high_speed_tracking/main.cpp`
  - `src/localization_ros/src/state_estimator_node.cpp`
  - `src/localization_ros/src/location.cpp`
  - `src/vehicle_interface_ros/include/vehicle_interface_ros/Node.h`
  - `src/simulation_ros/src/simulation_node.cpp`
- Result:
  - Canonical and legacy topic/frame defaults are now maintained in one place and reused by core ROS entry nodes.
  - Reduced cross-module implicit coupling caused by duplicated literal strings.

L. Unified simulation cone confidence scale to contract (`uint32 [0,1000]`)
- Evidence:
  - `src/simulation_ros/src/simulation_node.cpp` (`EncodeConfidenceScaled`, cone confidence publish path)
  - `src/planning_ros/include/planning_ros/contract_utils.hpp` (decode rule `/1000.0`)
- Result:
  - Simulation output now matches planning/localization confidence contract, avoiding hidden semantic mismatch (`*100` vs `*1000`).

M. Collapsed mission subsystem wiring into shared stack launch (de-duplication)
- Evidence:
  - `src/fsd_launch/launch/subsystems/mission_stack.launch`
  - `src/fsd_launch/launch/trackdrive.launch`
  - `src/fsd_launch/launch/autocross.launch`
  - `src/fsd_launch/launch/skidpad.launch`
  - `src/fsd_launch/launch/acceleration.launch`
  - `src/fsd_launch/launch/ebs_test.launch`
- Result:
  - Perception/localization/planning/control/vehicle orchestration now has a single wiring source.
  - Mission files preserve behavior differences only via explicit parameters (`mode/mission/planner/control_mode/overlay`), reducing copy-paste coupling risk.

N. Applied shared mission stack to full simulation chain
- Evidence:
  - `src/fsd_launch/launch/subsystems/mission_stack.launch` (`enable_perception/enable_localization/enable_planning` switches)
  - `src/fsd_launch/launch/simulation/full_simulation.launch`
- Result:
  - Full simulation now reuses the same orchestration component and explicitly disables localization in-stack (`enable_localization:=false`) while keeping planner/control semantics.
  - Further reduced launch-level branch duplication between sim and non-sim mission composition.

O. P2/C1 second phase: consolidated duplicated PerfStats implementation into shared component
- Evidence:
  - `src/autodrive_msgs/include/autodrive_msgs/perf_stats.hpp`
  - `src/planning_ros/include/high_speed_tracking/utils/PerfStats.hpp`
  - `src/perception_ros/include/perception_ros/perf_stats.hpp`
- Result:
  - `planning_ros` and `perception_ros` now reuse the same PerfStats implementation via thin compatibility aliases.
  - Eliminated duplicated metric-statistics logic while keeping existing call sites and tests API-compatible.

P. P2/C2-C3 guardrails: added deprecation/default-contract gate and CI integration
- Evidence:
  - `scripts/check_deprecation_contracts.sh`
  - `.github/workflows/architecture-contract.yml` (step: `Run deprecation contract checks`)
- Scope:
  - Verifies legacy topic toggles default-off in mission and stack launch files.
  - Verifies file-fallback defaults remain disabled in control entry points.
  - Verifies visualization `compat/enable_legacy_partial_full` default-off behavior.
  - Verifies standard missions are composed through `mission_stack.launch` (no duplicated direct subsystem wiring).
- Result:
  - C2/C3 policies moved from documentation-only to executable gate checks.

Q. Added runtime deprecation warnings when compatibility paths are enabled
- Evidence:
  - `src/control_ros/src/control_node.cpp`
  - `src/vehicle_interface_ros/src/Node.cpp`
- Result:
  - Enabling legacy topic aliases now emits explicit warning with planned removal date (`2026-06-30`).
  - Enabling file-based compatibility path now emits explicit warning with planned removal date (`2026-09-30`).

R. Refactored `localization_ros::LocPerfStats` onto a shared generic stats skeleton
- Evidence:
  - `src/autodrive_msgs/include/autodrive_msgs/perf_stats_skeleton.hpp`
  - `src/autodrive_msgs/include/autodrive_msgs/perf_stats.hpp`
  - `src/localization_ros/include/localization_ros/localization_perf_stats.hpp`
  - `src/localization_ros/test/test_localization_perf_stats.cpp`
  - `src/localization_ros/CMakeLists.txt`
  - `src/localization_ros/package.xml`
  - `src/planning_ros/include/high_speed_tracking/utils/PerfStats.hpp`
  - `src/perception_ros/include/perception_ros/perf_stats.hpp`
- Result:
  - Introduced reusable rolling-statistics skeleton (`RollingStatsWindow`) and shared metric type.
  - `LocPerfStats` now reuses the same core percentile/rolling-window engine while keeping localization-specific sample fields and snapshot schema unchanged.
  - Added regression tests that explicitly cover localization-specific metric fields (`t_opt_ms/t_preint_ms/t_assoc_ms/t_prune_ms/cone_match_ratio/gnss_availability`) to verify schema/behavior preservation.
  - Existing planning/perception `PerfStats` compatibility headers now alias the shared implementation, removing duplicated statistic core logic.

S. Added dedicated perf-stats contract gate (local + CI)
- Evidence:
  - `scripts/check_perf_stats_contracts.sh`
  - `.github/workflows/architecture-contract.yml` (step: `Run perf stats contract checks`)
- Result:
  - Perf-stats contract tests no longer depend on `catkin run_tests` PTY availability.
  - `localization_ros` and `perception_ros` shared-statistics regression is now continuously guarded in CI.

## Build / Syntax Validation

- XML syntax checks: passed (`xmllint --noout` on modified launch files).
- Build check: passed
  - `catkin build --no-status --summarize autodrive_msgs control_ros planning_ros localization_ros vehicle_interface_ros simulation_ros fsd_launch perception_ros`
- Contract static check: passed
  - `scripts/check_topic_contracts.sh`
- Deprecation/default-contract check: passed
  - `scripts/check_deprecation_contracts.sh`
- Deprecated-path warning instrumentation build regression: passed
  - `catkin build --no-status --summarize control_ros vehicle_interface_ros`
- LocPerfStats skeleton refactor regression: passed
  - `catkin build --no-status --summarize autodrive_msgs planning_ros perception_ros localization_ros`
  - `scripts/check_topic_contracts.sh`
  - `scripts/check_deprecation_contracts.sh`
- LocPerfStats and PerfStats unit tests (manual binary run due sandbox PTY limits): passed
  - `build/localization_ros && make localization_ros_perf_stats_test && devel/.private/localization_ros/lib/localization_ros/localization_ros_perf_stats_test` (`3/3` tests passed)
  - `build/perception_ros && make perception_ros_perf_stats_test && devel/.private/perception_ros/lib/perception_ros/perception_ros_perf_stats_test` (`2/2` tests passed)
- Perf-stats contract gate script: passed
  - `scripts/check_perf_stats_contracts.sh`
- Mission stack refactor regression check: passed
  - `catkin build --no-status --summarize fsd_launch`
  - `scripts/check_topic_contracts.sh`
- Full simulation stack migration check: passed
  - `scripts/check_topic_contracts.sh` (`full_simulation` unified/none branches)
- Runtime smoke local check: blocked by sandbox permission (`PermissionError` in `netifaces.interfaces()` during roslaunch startup), script and CI stage are in place.
- Runtime smoke local check (non-strict): `SKIPPED` as designed under env limitation.
- Runtime smoke strict check (`RUNTIME_SMOKE_REQUIRED=1`): `FAILED` as designed under env limitation.
- `catkin run_tests perception_ros` in this sandbox failed before executing tests due environment PTY exhaustion (`OSError: out of pty devices`), not due compile/runtime regression in changed code paths.

## Remaining TODO (next incremental wave)

1. Observe first CI runs of `architecture-contract.yml` and tune smoke timeouts/node expectations if needed (stability pass).
