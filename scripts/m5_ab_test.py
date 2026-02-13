#!/usr/bin/env python3
"""M5 A/B 验收测试脚本"""

import rospy
from nav_msgs.msg import Path
from autodrive_msgs.msg import vehicle_cmd
from collections import deque
import sys

class MetricsCollector:
    def __init__(self):
        self.pathlimits_times = deque(maxlen=1000)
        self.cmd_times = deque(maxlen=1000)
        
    def pathlimits_callback(self, msg):
        self.pathlimits_times.append(rospy.Time.now().to_sec())
        
    def cmd_callback(self, msg):
        self.cmd_times.append(rospy.Time.now().to_sec())
        
    def calculate_metrics(self, times):
        if len(times) < 2:
            return 0, 0, 0
        duration = times[-1] - times[0]
        freq = (len(times) - 1) / duration if duration > 0 else 0
        intervals = [times[i+1] - times[i] for i in range(len(times)-1)]
        max_interval = max(intervals) if intervals else 0
        avg_interval = sum(intervals) / len(intervals) if intervals else 0
        return freq, max_interval, avg_interval
        
    def print_report(self):
        path_freq, path_max, path_avg = self.calculate_metrics(self.pathlimits_times)
        cmd_freq, cmd_max, cmd_avg = self.calculate_metrics(self.cmd_times)
        
        print("\n" + "="*60)
        print("M5 A/B 验收测试报告")
        print("="*60)
        print(f"\n【planning/pathlimits】")
        print(f"  平均频率: {path_freq:.2f} Hz (目标 >= 8 Hz)")
        print(f"  最大间隔: {path_max:.3f} s (目标 <= 0.25 s)")
        print(f"  平均间隔: {path_avg:.3f} s")
        print(f"  消息数量: {len(self.pathlimits_times)}")
        
        print(f"\n【vehcileCMDMsg】")
        print(f"  平均频率: {cmd_freq:.2f} Hz (目标 >= 10 Hz)")
        print(f"  最大间隔: {cmd_max:.3f} s (目标 <= 0.20 s)")
        print(f"  平均间隔: {cmd_avg:.3f} s")
        print(f"  消息数量: {len(self.cmd_times)}")
        
        print("\n" + "="*60)
        print("验收结果")
        print("="*60)
        
        passed = True
        if path_freq < 8:
            print("[FAIL] planning/pathlimits 频率低于 8 Hz")
            passed = False
        else:
            print("[PASS] planning/pathlimits 频率 >= 8 Hz")
            
        if path_max > 0.25:
            print("[FAIL] planning/pathlimits 最大间隔超过 0.25 s")
            passed = False
        else:
            print("[PASS] planning/pathlimits 最大间隔 <= 0.25 s")
            
        if cmd_freq < 10:
            print("[FAIL] vehcileCMDMsg 频率低于 10 Hz")
            passed = False
        else:
            print("[PASS] vehcileCMDMsg 频率 >= 10 Hz")
            
        if cmd_max > 0.20:
            print("[FAIL] vehcileCMDMsg 最大间隔超过 0.20 s")
            passed = False
        else:
            print("[PASS] vehcileCMDMsg 最大间隔 <= 0.20 s")
            
        print("="*60)
        if passed:
            print("✓ 总体结果: PASS")
        else:
            print("✗ 总体结果: FAIL")
        print("="*60 + "\n")
        
        return passed

def main():
    rospy.init_node('m5_ab_test')
    collector = MetricsCollector()
    rospy.Subscriber('/planning/pathlimits', Path, collector.pathlimits_callback)
    rospy.Subscriber('/vehcileCMDMsg', vehicle_cmd, collector.cmd_callback)
    print("\n开始 M5 A/B 验收测试，测试 30 秒...")
    rospy.sleep(30)
    result = collector.print_report()
    sys.exit(0 if result else 1)

if __name__ == '__main__':
    main()
