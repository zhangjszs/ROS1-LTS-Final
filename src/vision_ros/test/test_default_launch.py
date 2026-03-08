#!/usr/bin/env python3
import unittest
import roslaunch
import rospy
import os
import time

PKG = 'vision_ros'

class TestDefaultLaunch(unittest.TestCase):
    def test_default_launch_starts_python_node(self):
        # Test that default vision.launch starts Python node, not C++
        roslaunch_file = os.path.join(rospkg.RosPack().get_path('vision_ros'), 'launch', 'vision.launch')

        # Configure and start launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_file])
        launch.start()

        try:
            # Wait for node to start
            time.sleep(3)

            # Check running nodes
            nodes = rospy.get_node_uri()
            all_nodes = rospy.get_published_topics()

            # Check if Python vision node is running
            python_node_running = False
            cpp_node_running = False

            for node_name in rospy.get_node_names():
                if 'vision_node_py' in node_name or '/vision_node' in node_name:
                    # Check node type by looking at cmdline
                    try:
                        import psutil
                        for proc in psutil.process_iter(['cmdline']):
                            try:
                                cmdline = proc.info['cmdline']
                                if cmdline and any('vision_node_py.py' in part for part in cmdline):
                                    python_node_running = True
                                if cmdline and any('vision_node' in part and not '.py' in part for part in cmdline):
                                    cpp_node_running = True
                            except:
                                pass
                    except:
                        # If psutil not available, just check node name convention
                        if 'py' in node_name.lower():
                            python_node_running = True
                        else:
                            cpp_node_running = True

            self.assertTrue(python_node_running, "Python vision node should be running by default")
            self.assertFalse(cpp_node_running, "C++ vision node should NOT be running by default")

        finally:
            # Shutdown launch
            launch.shutdown()

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_default_launch', TestDefaultLaunch)
