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
            f'<arg name="decision_fusion_budget_sec" default="{expected_default}"', content)
        self.assertIn(
            '<arg name="decision_fusion_budget_sec" value="$(arg decision_fusion_budget_sec)"', content)

    def test_perception_subsystem_launches_adapter(self):
        content = self._read("src/fsd_launch/launch/subsystems/perception.launch")
        self.assertIn('type="cone_detection_adapter_node"', content)
        self.assertIn('name="decision_cone_topic"', content)
        self.assertIn('topics/output', content)
        self.assertIn('topics/trace', content)
        self.assertIn('perception/decision/detections', content)
        self.assertNotIn('decision_cone_holdoff_sec', content)
        self.assertNotIn('raw_holdoff_sec', content)

    def test_mission_stack_routes_mainline_topic(self):
        content = self._read("src/fsd_launch/launch/subsystems/mission_stack.launch")
        self.assertIn('name="decision_cone_topic"', content)
        self.assertIn('name="mainline_cone_topic"', content)
        self.assertIn('value="$(arg mainline_cone_topic)"', content)

    def test_mission_stack_propagates_decision_fusion_budget(self):
        content = self._read("src/fsd_launch/launch/subsystems/mission_stack.launch")
        self.assertIn('<arg name="decision_fusion_budget_sec" default="0.15"/>', content)
        self.assertIn(
            '<arg name="decision_fusion_budget_sec" value="$(arg decision_fusion_budget_sec)"/>',
            content)

    def test_location_and_planning_accept_external_cone_topic(self):
        location = self._read("src/localization_ros/launch/location.launch")
        planning = self._read("src/planning_ros/launch/planning_pipeline.launch")
        self.assertIn('name="cone_topic"', location)
        self.assertIn('name="topics/cone" value="$(arg cone_topic)"', location)
        self.assertIn('name="input_cone_topic"', planning)
        self.assertIn('name="topics/cone" value="$(arg input_cone_topic)"', planning)

    def test_perception_propagates_decision_fusion_budget_to_lidar_cluster(self):
        perception = self._read("src/fsd_launch/launch/subsystems/perception.launch")
        lidar_cluster = self._read("src/perception_ros/launch/lidar_cluster.launch")
        self.assertIn('<arg name="decision_fusion_budget_sec" default="0.15"', perception)
        self.assertIn(
            '<arg name="decision_fusion_budget_sec" value="$(arg decision_fusion_budget_sec)"/>',
            perception)
        self.assertIn('<arg name="decision_fusion_budget_sec" default="0.15"', lidar_cluster)
        self.assertIn(
            '<param name="vision_inject/max_age_sec" value="$(arg decision_fusion_budget_sec)" />',
            lidar_cluster)

    def test_adapter_only_launch_defaults_legacy_budget_to_015(self):
        adapter_only = self._read("src/perception_ros/launch/cone_detection_adapter_only.launch")
        self.assertIn('<arg name="legacy_budget_sec" default="0.15"/>', adapter_only)

    def test_replay_suite_defaults_legacy_budget_to_015(self):
        replay_suite = self._read("scripts/run_adapter_replay_suite.sh")
        self.assertIn('LEGACY_BUDGET_SEC="${LEGACY_BUDGET_SEC:-0.15}"', replay_suite)

    def test_cross_module_topic_contracts_are_absolute(self):
        contract = self._read("src/fsd_common/include/fsd_common/topic_contract.hpp")
        self.assertIn(
            'inline constexpr const char *kFusedConeDetections = "/perception/fusion/detections";',
            contract)
        self.assertIn(
            'inline constexpr const char *kVisionDetections = "/perception/vision/detections";',
            contract)
        self.assertIn(
            'inline constexpr const char *kVisionDebugImage = "/perception/vision/debug_image";',
            contract)
        self.assertIn(
            'inline constexpr const char *kVisionDiagnostics = "/perception/vision/diagnostics";',
            contract)
        self.assertIn(
            'inline constexpr const char *kPerceptionDiagnostics = "/perception/diagnostics";',
            contract)

    def test_mission_entry_budget_defaults(self):
        self._assert_mission_budget("src/fsd_launch/launch/trackdrive.launch", "0.12")
        self._assert_mission_budget("src/fsd_launch/launch/autocross.launch", "0.12")
        self._assert_mission_budget("src/fsd_launch/launch/acceleration.launch", "0.10")
        self._assert_mission_budget("src/fsd_launch/launch/skidpad.launch", "0.15")


if __name__ == "__main__":
    import rostest

    rostest.rosrun("perception_ros", "test_mainline_adapter_launch_contract",
                   TestMainlineAdapterLaunchContract)
