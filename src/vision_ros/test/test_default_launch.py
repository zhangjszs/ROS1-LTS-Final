#!/usr/bin/env python3
import os
import unittest

import defusedxml.ElementTree as ET

import rospkg

PKG = "vision_ros"


class TestDefaultLaunch(unittest.TestCase):
    def test_default_launch_file_exists(self):
        roslaunch_file = os.path.join(
            rospkg.RosPack().get_path("vision_ros"), "launch", "vision.launch"
        )
        self.assertTrue(os.path.exists(roslaunch_file))

    def test_default_launch_valid_xml(self):
        roslaunch_file = os.path.join(
            rospkg.RosPack().get_path("vision_ros"), "launch", "vision.launch"
        )
        tree = ET.parse(roslaunch_file)
        root = tree.getroot()
        self.assertIsNotNone(root)


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_default_launch", TestDefaultLaunch)
