#!/usr/bin/env python3
"""
INS Message Bridge (Updated for HUAT_InsP2)
将 ins/ASENSING 消息转换为 autodrive_msgs/HUAT_InsP2
包含单位转换和符号修正，符合 INS5711DAA 手册规范
"""

import rospy
import math
from ins.msg import ASENSING
from autodrive_msgs.msg import HUAT_InsP2


class InsBridge:
    def __init__(self):
        # 单位转换常量
        self.DEG2RAD = math.pi / 180.0
        self.G = 9.7883105  # 重力加速度 (m/s²), 来自 INS5711DAA 手册
        
        # 发布 HUAT_InsP2 格式
        self.pub = rospy.Publisher('sensors/ins', HUAT_InsP2, queue_size=10)
        self.sub = rospy.Subscriber('/INS/ASENSING_INS', ASENSING, self.callback)

        rospy.loginfo("[InsBridge] Bridging /INS/ASENSING_INS -> sensors/ins (HUAT_InsP2)")
        rospy.loginfo("[InsBridge] Unit conversions: deg/s->rad/s, g->m/s²")

    def callback(self, msg):
        """
        转换 ASENSING 消息为 HUAT_InsP2
        
        注意事项:
        - 角速度: 原始 deg/s -> 转换为 rad/s
        - 加速度: 原始 g -> 转换为 m/s²  
        - 速度: ground_velocity(向下) -> Vd(向下), 直接使用
        """
        out = HUAT_InsP2()
        
        # Header: 使用 rospy.Time.now()
        # 当 use_sim_time=true 且 rosbag 使用 --clock 时，
        # rospy.Time.now() 会返回模拟时间（与点云时间戳同步）
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = "imu"
        
        # GPS 时间
        out.Week = int(msg.gps_week_number) & 0xFFFF  # 转 uint16
        out.Time = msg.sec_of_week
        
        # 姿态角 (度) - 直接复制，已经是度
        out.Heading = msg.azimuth
        out.Pitch = msg.pitch
        out.Roll = msg.roll
        
        # 角速度 (deg/s -> rad/s)
        # ASENSING 可能已经是 rad/s，如果是 deg/s 则需转换
        # 根据你的实际数据判断，这里假设是 deg/s
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
        # ASENSING: north_velocity, east_velocity, ground_velocity(向下)
        # HUAT_InsP2: Vn, Ve, Vd(向下)
        out.Ve = msg.east_velocity
        out.Vn = msg.north_velocity
        out.Vd = msg.ground_velocity  # 向下为正
        
        # 双天线基线 (如果有的话，否则设为0)
        out.Base = 0.0  # ASENSING 消息中无此字段
        
        # 状态信息
        out.NSV1 = msg.numsv
        out.NSV2 = msg.numsv  # 如果只有一个值，两个都设为相同
        out.Status = self._convert_ins_status(msg.ins_status)
        out.Age = 0  # ASENSING 消息中无差分龄期字段，设为0
        out.War = 0  # 警告标志，根据需要设置
        
        self.pub.publish(out)
    
    def _convert_ins_status(self, asensing_status):
        """
        转换 ASENSING 的 ins_status 为 HUAT_InsP2 的 Status
        
        HUAT_InsP2 Status:
        - 0 = NONE (未初始化)
        - 1 = 姿态初始化完成
        - 2 = 组合导航正常
        
        根据你的 ASENSING 消息定义调整此映射逻辑
        """
        # 示例映射（需要根据实际 ASENSING 定义调整）
        if asensing_status == 0:
            return 0  # NONE
        elif asensing_status == 1:
            return 1  # 姿态初始化
        else:
            return 2  # 组合导航正常


if __name__ == '__main__':
    rospy.init_node('ins_bridge')
    bridge = InsBridge()
    rospy.spin()
