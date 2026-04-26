#!/usr/bin/env python3
"""Publish dummy HUAT_VisionDetections synced to point cloud timestamps.

This guarantees message_filters sync pairs are formed so the fusion path
is exercised even without a real vision model.
"""

import rospy
from sensor_msgs.msg import PointCloud2

from autodrive_msgs.msg import HUAT_VisionDetections


def on_cloud(cloud_msg):
    msg = HUAT_VisionDetections()
    msg.header = cloud_msg.header
    msg.header.frame_id = "camera"
    # Empty detections: sync pair exists but no vision cones to match.
    # With camera_info missing, this triggers CAMERA_INFO_MISSING status.
    pub.publish(msg)


rospy.init_node("fake_vision_publisher")
pub = rospy.Publisher("/vision/detections", HUAT_VisionDetections, queue_size=10)
rospy.Subscriber("/velodyne_points", PointCloud2, on_cloud)
rospy.spin()
