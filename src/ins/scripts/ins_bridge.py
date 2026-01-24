#!/usr/bin/env python3
"""
INS Message Bridge
将 ins/ASENSING 消息转换为 autodrive_msgs/HUAT_Asensing
"""

import rospy
from ins.msg import ASENSING
from autodrive_msgs.msg import HUAT_Asensing


class InsBridge:
    def __init__(self):
        self.pub = rospy.Publisher('/pbox_pub/Ins', HUAT_Asensing, queue_size=10)
        self.sub = rospy.Subscriber('/INS/ASENSING_INS', ASENSING, self.callback)
        rospy.loginfo("[InsBridge] Bridging /INS/ASENSING_INS -> /pbox_pub/Ins")

    def callback(self, msg):
        out = HUAT_Asensing()
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        out.north_velocity = msg.north_velocity
        out.east_velocity = msg.east_velocity
        out.ground_velocity = msg.ground_velocity
        out.roll = msg.roll
        out.pitch = msg.pitch
        out.azimuth = msg.azimuth
        out.x_angular_velocity = msg.x_angular_velocity
        out.y_angular_velocity = msg.y_angular_velocity
        out.z_angular_velocity = msg.z_angular_velocity
        out.x_acc = msg.x_acc
        out.y_acc = msg.y_acc
        out.z_acc = msg.z_acc
        out.latitude_std = msg.latitude_std
        out.longitude_std = msg.longitude_std
        out.altitude_std = msg.altitude_std
        out.north_velocity_std = msg.north_velocity_std
        out.east_velocity_std = msg.east_velocity_std
        out.ground_velocity_std = msg.ground_velocity_std
        out.roll_std = msg.roll_std
        out.pitch_std = msg.pitch_std
        out.azimuth_std = msg.azimuth_std
        out.ins_status = msg.ins_status
        out.position_type = msg.position_type
        out.sec_of_week = msg.sec_of_week
        out.gps_week_number = msg.gps_week_number
        out.temperature = msg.temperature
        out.wheel_speed_status = msg.wheel_speed_status
        out.heading_type = msg.heading_type
        out.numsv = msg.numsv
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('ins_bridge')
    bridge = InsBridge()
    rospy.spin()
