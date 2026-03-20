#!/usr/bin/env python3
"""
inject_scenario.py - 定位系统场景注入测试工具
用于向定位系统注入各种测试场景，验证系统鲁棒性
"""

import rospy
import time
import argparse
import numpy as np
from autodrive_msgs.msg import HUAT_CarState, HUAT_ConeDetections
from geometry_msgs.msg import Point32

def inject_gps_loss(duration=5.0):
    """注入GPS丢失场景，停止发布CarState一段时间"""
    print(f"[+] 注入GPS丢失场景，持续时间: {duration}s")
    pub = rospy.Publisher('/localization/car_state', HUAT_CarState, queue_size=10)
    rospy.sleep(0.5)

    # 停止发布消息来模拟GPS丢失
    start_time = time.time()
    while time.time() - start_time < duration and not rospy.is_shutdown():
        # 不发布任何消息，模拟丢失
        rospy.sleep(0.1)

    print("[+] GPS丢失场景结束")

def inject_cone_noise(noise_std=0.3, duration=10.0):
    """向锥桶检测注入噪声"""
    print(f"[+] 注入锥桶检测噪声，标准差: {noise_std}m，持续时间: {duration}s")

    def callback(msg):
        # 添加噪声
        noisy_msg = HUAT_ConeDetections()
        noisy_msg.header = msg.header

        for cone in msg.points:
            noisy_cone = Point32()
            noisy_cone.x = cone.x + np.random.normal(0, noise_std)
            noisy_cone.y = cone.y + np.random.normal(0, noise_std)
            noisy_cone.z = cone.z
            noisy_msg.points.append(noisy_cone)

        # 复制其他字段
        noisy_msg.confidences = msg.confidences
        noisy_msg.color_types = msg.color_types
        noisy_msg.track_ids = msg.track_ids

        pub.publish(noisy_msg)

    # 订阅原始检测，发布带噪声的检测
    sub = rospy.Subscriber('/perception/lidar_cluster/detections', HUAT_ConeDetections, callback)
    pub = rospy.Publisher('/perception/lidar_cluster/detections_noisy', HUAT_ConeDetections, queue_size=10)

    rospy.sleep(duration)
    sub.unregister()
    print("[+] 锥桶噪声注入结束")

def inject_wrong_cone_colors(swap_prob=0.2, duration=10.0):
    """注入锥桶颜色错误"""
    print(f"[+] 注入锥桶颜色错误，交换概率: {swap_prob*100}%，持续时间: {duration}s")

    def callback(msg):
        modified_msg = HUAT_ConeDetections()
        modified_msg.header = msg.header
        modified_msg.points = msg.points
        modified_msg.confidences = msg.confidences
        modified_msg.track_ids = msg.track_ids

        for color in msg.color_types:
            if np.random.random() < swap_prob:
                # 随机交换颜色，红蓝互换
                if color == 0:  # BLUE -> RED
                    modified_msg.color_types.append(3)
                elif color == 3:  # RED -> BLUE
                    modified_msg.color_types.append(0)
                else:
                    modified_msg.color_types.append(color)
            else:
                modified_msg.color_types.append(color)

        pub.publish(modified_msg)

    sub = rospy.Subscriber('/perception/lidar_cluster/detections', HUAT_ConeDetections, callback)
    pub = rospy.Publisher('/perception/lidar_cluster/detections_bad_color', HUAT_ConeDetections, queue_size=10)

    rospy.sleep(duration)
    sub.unregister()
    print("[+] 颜色错误注入结束")

def main():
    parser = argparse.ArgumentParser(description='定位系统场景注入测试工具')
    parser.add_argument('--scenario', type=str, required=True,
                        choices=['gps_loss', 'cone_noise', 'wrong_color'],
                        help='要注入的场景类型')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='场景持续时间(秒)，默认10s')
    parser.add_argument('--noise-std', type=float, default=0.3,
                        help='锥桶位置噪声标准差(米)，默认0.3m')
    parser.add_argument('--swap-prob', type=float, default=0.2,
                        help='颜色交换概率，默认0.2')

    args = parser.parse_args()

    rospy.init_node('scenario_injector', anonymous=True)
    print("=== 定位场景注入工具 ===")

    try:
        if args.scenario == 'gps_loss':
            inject_gps_loss(args.duration)
        elif args.scenario == 'cone_noise':
            inject_cone_noise(args.noise_std, args.duration)
        elif args.scenario == 'wrong_color':
            inject_wrong_cone_colors(args.swap_prob, args.duration)
    except KeyboardInterrupt:
        print("\n[!] 用户中断")

    print("[*] 测试结束")

if __name__ == "__main__":
    main()
