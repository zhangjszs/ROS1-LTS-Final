#!/usr/bin/env python3
"""
INS Message Bridge (Updated for HUAT_InsP2)
将 autodrive_msgs/HUAT_Asensing 消息转换为 autodrive_msgs/HUAT_InsP2
包含单位转换和符号修正，符合 INS5711DAA 手册规范

Migrated from ins package to vehicle_interface_ros (P2-1).
"""

import rospy
import math
from autodrive_msgs.msg import HUAT_Asensing, HUAT_InsP2


class InsBridge:
    def __init__(self):
        # 单位转换常量
        self.DEG2RAD = math.pi / 180.0
        self.G = 9.7883105  # 重力加速度 (m/s²), 来自 INS5711DAA 手册

        # 发布 HUAT_InsP2 格式
        self.pub = rospy.Publisher('sensors/ins', HUAT_InsP2, queue_size=10)
        self.sub = rospy.Subscriber('/INS/ASENSING_INS', HUAT_Asensing, self.callback)

        rospy.loginfo("[InsBridge] Bridging /INS/ASENSING_INS -> sensors/ins (HUAT_InsP2)")
        rospy.loginfo("[InsBridge] Unit conversions: deg/s->rad/s, g->m/s²")

    def callback(self, msg):
        """
        转换 HUAT_Asensing 消息为 HUAT_InsP2

        注意事项:
        - 角速度: 原始 deg/s -> 转换为 rad/s
        - 加速度: 原始 g -> 转换为 m/s²
        - 速度: ground_velocity(向下) -> Vd(向下), 直接使用
        """
        out = HUAT_InsP2()

        out.header.stamp = rospy.Time.now()
        out.header.frame_id = "imu"

        # GPS 时间
        out.Week = int(msg.gps_week_number) & 0xFFFF
        out.Time = msg.sec_of_week

        # 姿态角 (度)
        out.Heading = msg.azimuth
        out.Pitch = msg.pitch
        out.Roll = msg.roll

        # 角速度 (deg/s -> rad/s)
        out.gyro_x = msg.x_angular_velocity * self.DEG2RAD
        out.gyro_y = msg.y_angular_velocity * self.DEG2RAD
        out.gyro_z = msg.z_angular_velocity * self.DEG2RAD

        # 加速度 (g -> m/s²)
        # ASENSING 输出单位为 g，坐标系为 FLU (Up正)
        # HUAT_InsP2 定义为 FRD (Down正)，需要对 Z 轴取反
        out.acc_x = msg.x_acc * self.G
        out.acc_y = msg.y_acc * self.G
        out.acc_z = -msg.z_acc * self.G  # FLU -> FRD: Z 取反

        # 位置 (WGS84)
        out.Lat = msg.latitude
        out.Lon = msg.longitude
        out.Altitude = msg.altitude

        # 速度 (m/s, NED坐标系)
        out.Ve = msg.east_velocity
        out.Vn = msg.north_velocity
        out.Vd = msg.ground_velocity  # 向下为正

        # 双天线基线
        out.Base = 0.0  # ASENSING 消息中无此字段

        # 状态信息
        out.NSV1 = msg.numsv
        out.NSV2 = msg.numsv
        out.Status = self._convert_ins_status(msg.ins_status)
        out.Age = 0
        out.War = 0

        self.pub.publish(out)

    def _convert_ins_status(self, asensing_status):
        """
        转换 ASENSING 的 ins_status 为 HUAT_InsP2 的 Status
        0 = NONE, 1 = 姿态初始化完成, 2 = 组合导航正常
        """
        if asensing_status == 0:
            return 0
        elif asensing_status == 1:
            return 1
        else:
            return 2


if __name__ == '__main__':
    rospy.init_node('ins_bridge')
    bridge = InsBridge()
    rospy.spin()
