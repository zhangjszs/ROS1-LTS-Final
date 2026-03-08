#!/usr/bin/env python3
import unittest
import rospy
import os
from rospkg import RosPack

PKG = 'perception_ros'

class TestVisionInjectDefault(unittest.TestCase):
    def test_vision_inject_enabled_by_default(self):
        # Test that vision_inject is enabled by default in lidar_base.yaml
        config_file = os.path.join(RosPack().get_path('perception_ros'), 'config', 'lidar_base.yaml')

        import yaml
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        # Check vision_inject enabled flag
        vision_inject_config = config.get('lidar_cluster_node', {}).get('vision_inject', {})
        enabled = vision_inject_config.get('enabled', False)

        self.assertTrue(enabled, "vision_inject should be enabled by default in lidar_base.yaml")
        self.assertGreater(vision_inject_config.get('min_confidence', 0), 0, "min_confidence should be set")
        self.assertGreater(vision_inject_config.get('max_age_sec', 0), 0, "max_age_sec should be set")

    def test_launch_file_enables_vision_inject(self):
        # Test that perception launch file doesn't disable vision_inject
        launch_file = os.path.join(RosPack().get_path('fsd_launch'), 'launch', 'subsystems', 'perception.launch')

        with open(launch_file, 'r') as f:
            content = f.read()

        # Check that vision_inject/enabled is not set to false anywhere
        self.assertFalse('vision_inject/enabled: false' in content.lower(), "Launch file should not disable vision_inject by default")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_vision_inject_default', TestVisionInjectDefault)
