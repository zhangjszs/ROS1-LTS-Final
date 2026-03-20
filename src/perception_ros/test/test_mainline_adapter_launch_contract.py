#!/usr/bin/env python3
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]


class TestMainlineAdapterLaunchContract(unittest.TestCase):
    def _read(self, relpath):
        return (REPO_ROOT / relpath).read_text()

    def _assert_mission_budget(self, relpath, expected_default):
        content = self._read(relpath)
        self.assertIn(
            f'<arg name="decision_fusion_budget_sec" default="{expected_default}"',
            content,
        )
        self.assertIn(
            '<arg name="decision_fusion_budget_sec" value="$(arg decision_fusion_budget_sec)"',
            content,
        )

    def test_perception_subsystem_launches_adapter(self):
        content = self._read("src/fsd_launch/launch/subsystems/perception.launch")
        self.assertIn('type="cone_detection_adapter_node"', content)
        self.assertIn('name="decision_cone_topic"', content)
        self.assertIn("topics/output", content)
        self.assertIn("topics/trace", content)
        self.assertIn("perception/decision/detections", content)
        self.assertNotIn("decision_cone_holdoff_sec", content)
        self.assertNotIn("raw_holdoff_sec", content)

    def test_mission_stack_routes_mainline_topic(self):
        content = self._read("src/fsd_launch/launch/subsystems/mission_stack.launch")
        self.assertIn('name="decision_cone_topic"', content)
        self.assertIn('name="mainline_cone_topic"', content)
        self.assertIn('value="$(arg mainline_cone_topic)"', content)
        self.assertIn('name="pathlimits_v2_topic"', content)
        self.assertIn('name="planning_enable_pathlimits_v1_publish"', content)
        self.assertIn('name="planning_enable_pathlimits_v2_publish"', content)

    def test_mission_stack_propagates_decision_fusion_budget(self):
        content = self._read("src/fsd_launch/launch/subsystems/mission_stack.launch")
        self.assertIn('<arg name="decision_fusion_budget_sec" default="0.15"/>', content)
        self.assertIn(
            '<arg name="decision_fusion_budget_sec" value="$(arg decision_fusion_budget_sec)"/>',
            content,
        )

    def test_location_and_planning_accept_external_cone_topic(self):
        location = self._read("src/localization_ros/launch/location.launch")
        planning = self._read("src/planning_ros/launch/planning_pipeline.launch")
        self.assertIn('name="cone_topic"', location)
        self.assertIn('name="topics/cone" value="$(arg cone_topic)"', location)
        self.assertIn('name="input_cone_topic"', planning)
        self.assertIn('name="topics/cone" value="$(arg input_cone_topic)"', planning)
        self.assertIn('name="output_pathlimits_v2_topic"', planning)
        self.assertIn(
            'name="topics/pathlimits_v2" value="$(arg output_pathlimits_v2_topic)"', planning
        )
        self.assertIn('name="compat/enable_pathlimits_v1_publish"', planning)
        self.assertIn('name="compat/enable_pathlimits_v2_publish"', planning)

    def test_localization_backend_cutover_args_are_wired(self):
        mission_stack = self._read("src/fsd_launch/launch/subsystems/mission_stack.launch")
        localization_subsystem = self._read("src/fsd_launch/launch/subsystems/localization.launch")
        location_launch = self._read("src/localization_ros/launch/location.launch")

        self.assertIn('name="backend" default="mapper"', mission_stack)
        self.assertIn('name="publish_dual_backends" default="false"', mission_stack)
        self.assertIn('name="fg_shadow_mode" default="true"', mission_stack)
        self.assertIn('name="fg_mainline_enable_mapper_fallback" default="true"', mission_stack)
        self.assertIn('name="backend" value="$(arg backend)"', mission_stack)
        self.assertIn(
            'name="publish_dual_backends" value="$(arg publish_dual_backends)"',
            mission_stack,
        )

        self.assertIn('name="fg_shadow_mode" value="$(arg fg_shadow_mode)"', localization_subsystem)
        self.assertIn(
            'name="fg_mainline_enable_mapper_fallback" value="$(arg fg_mainline_enable_mapper_fallback)"',
            localization_subsystem,
        )
        self.assertIn('name="backend" value="$(arg backend)"', localization_subsystem)
        self.assertIn(
            'name="publish_dual_backends" value="$(arg publish_dual_backends)"',
            localization_subsystem,
        )

        self.assertIn('name="backend" value="$(arg backend)"', location_launch)
        self.assertIn('name="fg/shadow_mode" value="$(arg fg_shadow_mode)"', location_launch)
        self.assertIn(
            'name="fg/mainline_enable_mapper_fallback" value="$(arg fg_mainline_enable_mapper_fallback)"',
            location_launch,
        )

    def test_perception_propagates_decision_fusion_budget_to_lidar_cluster(self):
        perception = self._read("src/fsd_launch/launch/subsystems/perception.launch")
        lidar_cluster = self._read("src/perception_ros/launch/lidar_cluster.launch")
        self.assertIn('<arg name="decision_fusion_budget_sec" default="0.15"', perception)
        self.assertIn(
            '<arg name="decision_fusion_budget_sec" value="$(arg decision_fusion_budget_sec)"/>',
            perception,
        )
        self.assertIn('<arg name="decision_fusion_budget_sec" default="0.15"', lidar_cluster)
        self.assertIn(
            '<param name="vision_inject/max_age_sec" value="$(arg decision_fusion_budget_sec)" />',
            lidar_cluster,
        )

    def test_adapter_only_launch_defaults_legacy_budget_to_015(self):
        adapter_only = self._read("src/perception_ros/launch/cone_detection_adapter_only.launch")
        self.assertIn('<arg name="legacy_budget_sec" default="0.15"/>', adapter_only)

    def test_replay_suite_defaults_legacy_budget_to_015(self):
        replay_suite = self._read("scripts/run_adapter_replay_suite.sh")
        self.assertIn('LEGACY_BUDGET_SEC="${LEGACY_BUDGET_SEC:-0.15}"', replay_suite)

    def test_cross_module_topic_contracts_are_absolute(self):
        contract = self._read("src/fsd_common/include/fsd_common/topic_contract.hpp")
        self.assertIn(
            'kFusedConeDetections = "/perception/fusion/detections"',
            contract,
        )
        self.assertIn(
            'kVisionDetections = "/perception/vision/detections"',
            contract,
        )
        self.assertIn(
            'kVisionDebugImage = "/perception/vision/debug_image"',
            contract,
        )
        self.assertIn(
            'kVisionDiagnostics = "/perception/vision/diagnostics"',
            contract,
        )
        self.assertIn(
            'kPerceptionDiagnostics = "/perception/diagnostics"',
            contract,
        )

    def test_mission_entry_budget_defaults(self):
        self._assert_mission_budget("src/fsd_launch/launch/trackdrive.launch", "0.12")
        self._assert_mission_budget("src/fsd_launch/launch/autocross.launch", "0.12")
        self._assert_mission_budget("src/fsd_launch/launch/acceleration.launch", "0.10")
        self._assert_mission_budget("src/fsd_launch/launch/skidpad.launch", "0.15")

    def test_mission_entry_vision_image_topic_defaults(self):
        track = self._read("src/fsd_launch/launch/trackdrive.launch")
        accel = self._read("src/fsd_launch/launch/acceleration.launch")
        skidpad = self._read("src/fsd_launch/launch/skidpad.launch")

        self.assertIn('<arg name="vision_image_topic" default="/resize_img_out"', track)
        self.assertIn('<arg name="vision_image_topic" default="/resize_img_out"', accel)
        self.assertIn('<arg name="vision_image_topic" default="/resize_img_out"', skidpad)

    def test_mission_entries_expose_pathlimits_v2_cutover_switches(self):
        for relpath in (
            "src/fsd_launch/launch/trackdrive.launch",
            "src/fsd_launch/launch/autocross.launch",
            "src/fsd_launch/launch/acceleration.launch",
            "src/fsd_launch/launch/skidpad.launch",
        ):
            content = self._read(relpath)
            self.assertIn(
                '<arg name="control_pathlimits_v2_topic" default="planning/pathlimits_v2"', content
            )
            self.assertIn(
                '<arg name="control_enable_pathlimits_v1_subscribe" default="true"',
                content,
            )
            self.assertIn(
                '<arg name="control_enable_pathlimits_v2_subscribe" default="false"',
                content,
            )
            self.assertIn('<arg name="control_prefer_pathlimits_v2" default="false"', content)
            self.assertIn(
                '<arg name="control_enable_pathlimits_v2_auto_fallback" default="true"',
                content,
            )
            self.assertIn(
                '<arg name="control_pathlimits_source_stale_timeout_sec" default="-1.0"',
                content,
            )
            self.assertIn(
                '<arg name="control_pathlimits_v2_topic" value="$(arg control_pathlimits_v2_topic)"',
                content,
            )
            self.assertIn(
                '<arg name="control_enable_pathlimits_v1_subscribe" value="$(arg control_enable_pathlimits_v1_subscribe)"',
                content,
            )
            self.assertIn(
                '<arg name="control_enable_pathlimits_v2_subscribe" value="$(arg control_enable_pathlimits_v2_subscribe)"',
                content,
            )
            self.assertIn(
                '<arg name="control_prefer_pathlimits_v2" value="$(arg control_prefer_pathlimits_v2)"',
                content,
            )
            self.assertIn(
                '<arg name="control_enable_pathlimits_v2_auto_fallback" value="$(arg control_enable_pathlimits_v2_auto_fallback)"',
                content,
            )
            self.assertIn(
                '<arg name="control_pathlimits_source_stale_timeout_sec" value="$(arg control_pathlimits_source_stale_timeout_sec)"',
                content,
            )

    def test_control_mode_contract_accel_and_ebs_defaults(self):
        accel = self._read("src/fsd_launch/launch/acceleration.launch")
        ebs = self._read("src/fsd_launch/launch/ebs_test.launch")
        mission_stack = self._read("src/fsd_launch/launch/subsystems/mission_stack.launch")
        planning_subsystem = self._read("src/fsd_launch/launch/subsystems/planning.launch")

        self.assertIn('<arg name="control_mode" default="2"', accel)
        self.assertIn('<arg name="control_mode" value="$(arg control_mode)"', accel)
        self.assertIn('<arg name="control_mode" default="5"', ebs)
        self.assertIn('<arg name="planner" value="none"', ebs)
        self.assertIn('<arg name="control_mode_profile" value="track"', ebs)
        self.assertIn('<arg name="control_mode" default="4"', mission_stack)
        self.assertIn(
            '<arg name="output_pathlimits_v2_topic" default="planning/pathlimits_v2"',
            planning_subsystem,
        )
        self.assertIn(
            '<arg name="output_pathlimits_v2_topic" value="$(arg output_pathlimits_v2_topic)"',
            planning_subsystem,
        )

    def test_planning_pipeline_publishes_v1_v2_transition_chain(self):
        node_cpp = self._read("src/planning_ros/src/planning_pipeline_node.cpp")
        self.assertIn("HUAT_PathLimitsV2", node_cpp)
        self.assertIn("ConvertPathLimitsV1ToV2", node_cpp)
        self.assertIn("pathlimits_v2_pub_", node_cpp)
        self.assertIn("compat/enable_pathlimits_v2_publish", node_cpp)

    def test_control_launch_supports_pathlimits_v2_transition(self):
        control_subsystem = self._read("src/fsd_launch/launch/subsystems/control.launch")
        control_entry = self._read("src/control_ros/launch/controler.launch")
        mission_stack = self._read("src/fsd_launch/launch/subsystems/mission_stack.launch")

        self.assertIn('name="pathlimits_v2_topic"', control_subsystem)
        self.assertIn('name="enable_pathlimits_v1_subscribe"', control_subsystem)
        self.assertIn('name="enable_pathlimits_v2_subscribe"', control_subsystem)
        self.assertIn('name="prefer_pathlimits_v2"', control_subsystem)
        self.assertIn('name="enable_pathlimits_v2_auto_fallback"', control_subsystem)
        self.assertIn('name="pathlimits_source_stale_timeout_sec"', control_subsystem)
        self.assertIn('name="pathlimits_v2_topic"', control_entry)
        self.assertIn('name="compat/enable_pathlimits_v2_subscribe"', control_entry)
        self.assertIn('name="compat/enable_pathlimits_v2_auto_fallback"', control_entry)
        self.assertIn('name="compat/pathlimits_source_stale_timeout_sec"', control_entry)
        self.assertIn('name="control_pathlimits_v2_topic"', mission_stack)
        self.assertIn('name="control_enable_pathlimits_v2_auto_fallback"', mission_stack)
        self.assertIn('name="control_pathlimits_source_stale_timeout_sec"', mission_stack)

    def test_control_node_supports_pathlimits_v2_subscriber(self):
        control_node = self._read("src/control_ros/src/control_node.cpp")
        self.assertIn("HUAT_PathLimitsV2", control_node)
        self.assertIn("PathLimitsV2Callback", control_node)
        self.assertIn("pathlimits_v2_topic", control_node)
        self.assertIn("compat/enable_pathlimits_v2_subscribe", control_node)
        self.assertIn("compat/enable_pathlimits_v2_auto_fallback", control_node)
        self.assertIn("compat/pathlimits_source_stale_timeout_sec", control_node)
        self.assertIn("v1-fallback", control_node)

    def test_control_mode_constants_and_mapping_contract(self):
        mode_header = self._read("src/fsd_common/include/fsd_common/control_mode.hpp")
        control_node = self._read("src/control_ros/src/control_node.cpp")
        vehicle_if = self._read("src/vehicle_interface_ros/src/Node.cpp")

        self.assertIn("static constexpr int kLine = 2;", mode_header)
        self.assertIn("static constexpr int kEbs = 5;", mode_header)
        self.assertIn(
            "controller_ = std::make_unique<control_core::EbsController>();",
            control_node,
        )
        self.assertIn(
            "cmd.working_mode = (mode_ == fsd_common::ControlMode::kEbs) ? 2 : 1;",
            control_node,
        )
        self.assertIn("vehicle_tx_msg[7] = 2;    // EBS working mode", vehicle_if)

    def test_vehicle_interface_watchdog_checksum_contract(self):
        vehicle_if_header = self._read(
            "src/vehicle_interface_ros/include/vehicle_interface_ros/Node.h"
        )
        vehicle_if = self._read("src/vehicle_interface_ros/src/Node.cpp")
        self.assertIn("const double CMD_TIMEOUT_SEC = 0.1;", vehicle_if_header)
        self.assertIn("for (int i = 0; i < 10; ++i)", vehicle_if)
        self.assertIn("vehicle_tx_msg[10] = static_cast<uint8_t>(checksum & 0xFF);", vehicle_if)
        self.assertIn(
            "vehicle_tx_msg[11] = static_cast<uint8_t>((checksum >> 8) & 0xFF);",
            vehicle_if,
        )

    def test_color_semantics_gate_assets_exist(self):
        for mode in ("track", "accel", "skidpad"):
            env_file = (
                REPO_ROOT
                / f"perf_reports/baselines/perception/{mode}.color_semantics.thresholds.env"
            )
            self.assertTrue(
                env_file.exists(),
                f"Missing color semantics threshold file: {env_file}",
            )

        gate_script = REPO_ROOT / "scripts/check_color_semantics_regression_mode.sh"
        self.assertTrue(gate_script.exists(), f"Missing color semantics gate script: {gate_script}")

    def test_mode_regression_script_invokes_color_semantics_gate(self):
        mode_script = self._read("scripts/check_perception_regression_mode.sh")
        self.assertIn("--color-bag", mode_script)
        self.assertIn("check_color_semantics_regression_mode.sh", mode_script)

    def test_color_semantics_ci_workflow_invokes_gate(self):
        workflow = self._read(".github/workflows/perception_color_semantics_ci.yaml")
        self.assertIn("check_color_semantics_regression_mode.sh", workflow)
        self.assertIn("check_perception_regression_mode.sh", workflow)
        self.assertIn("evaluate_color_semantics_metrics.py", workflow)
        self.assertIn("test_mainline_adapter_launch_contract.py", workflow)

    def test_pathlimits_v2_cutover_replay_script_contract(self):
        script = self._read("scripts/validate_pathlimits_v2_cutover_replay.sh")
        self.assertIn("control_prefer_pathlimits_v2:=true", script)
        self.assertIn("control_enable_pathlimits_v2_auto_fallback:=true", script)
        self.assertIn("enable_pathlimits_v2_publish:=false", script)
        self.assertIn("/control/diagnostics", script)
        self.assertIn("active_pathlimits_source", script)
        self.assertIn("PathLimitsV2 recovered; switching back to V2 primary", script)
        self.assertIn("v2_seen_evidence", script)
        self.assertIn("diagnostics_count", script)

    def test_pathlimits_v2_cutover_matrix_script_contract(self):
        script = self._read("scripts/run_pathlimits_v2_cutover_matrix.sh")
        self.assertIn("validate_pathlimits_v2_cutover_replay.sh", script)
        self.assertIn("MISSIONS_CSV", script)
        self.assertIn("matrix.tsv", script)
        self.assertIn("summary.md", script)
        self.assertIn("matrix.json", script)


if __name__ == "__main__":
    import rostest

    rostest.rosrun(
        "perception_ros",
        "test_mainline_adapter_launch_contract",
        TestMainlineAdapterLaunchContract,
    )
