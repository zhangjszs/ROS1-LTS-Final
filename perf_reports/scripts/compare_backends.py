#!/usr/bin/env python3
"""
compare_backends.py - 定位后端对比工具
同时订阅 mapper 和 Factor Graph 两个后端的输出，对比定位精度和一致性。
"""

import argparse
import csv
import math
import time
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rospy
from autodrive_msgs.msg import HUAT_CarState


def build_csv_header():
    return "timestamp,pos_error_m,heading_error_deg,velocity_error_mps,time_diff_s"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="定位后端对比工具")
    parser.add_argument(
        "--mapper-topic",
        default="/localization/mapper/car_state",
        help="Mapper 后端输出话题",
    )
    parser.add_argument(
        "--fg-topic",
        default="/localization/fg/car_state",
        help="Factor Graph 后端输出话题",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="监听时长（秒），<=0 表示直到 Ctrl-C",
    )
    parser.add_argument(
        "--output",
        default="backend_comparison.csv",
        help="CSV 输出路径",
    )
    parser.add_argument(
        "--plot-output",
        default="",
        help="图表输出路径（默认与 CSV 同名 .png）",
    )
    parser.add_argument(
        "--max-sync-diff",
        type=float,
        default=0.1,
        help="允许匹配的最大时间差（秒）",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="仅输出 CSV，不生成图表",
    )
    return parser.parse_args(argv)


def _extract_state(msg):
    return (
        msg.header.stamp.to_sec(),
        msg.car_state.x,
        msg.car_state.y,
        msg.car_state.theta,
        msg.V,
    )


def _nearest_by_time(candidates, target_time):
    if not candidates:
        return None

    timestamps = [item[0] for item in candidates]
    idx = np.searchsorted(timestamps, target_time, side="left")
    if idx > 0 and (
        idx == len(timestamps)
        or abs(target_time - timestamps[idx - 1]) < abs(target_time - timestamps[idx])
    ):
        idx -= 1

    if idx >= len(candidates):
        return None
    return candidates[idx]


class BackendComparer:
    def __init__(
        self,
        output_file="backend_comparison.csv",
        mapper_topic="/localization/mapper/car_state",
        fg_topic="/localization/fg/car_state",
        max_sync_diff=0.1,
    ):
        self.mapper_states = deque(maxlen=1000)
        self.time_errors = []
        self.position_errors = []
        self.heading_errors = []
        self.velocity_errors = []
        self.max_sync_diff = max(0.0, float(max_sync_diff))
        self.output_file = output_file

        self._init_csv()

        self.sub_mapper = rospy.Subscriber(mapper_topic, HUAT_CarState, self.mapper_callback)
        self.sub_fg = rospy.Subscriber(fg_topic, HUAT_CarState, self.fg_callback)

        print("[+] 后端对比工具启动，正在监听两个定位后端的输出...")
        print("[+] 结果将保存到:", self.output_file)

    def _init_csv(self):
        with open(self.output_file, "w", newline="") as file_obj:
            file_obj.write(build_csv_header() + "\n")

    def _append_csv_row(self, timestamp, pos_error, heading_error, vel_error, time_diff):
        with open(self.output_file, "a", newline="") as file_obj:
            writer = csv.writer(file_obj)
            writer.writerow(
                [
                    f"{timestamp:.6f}",
                    f"{pos_error:.6f}",
                    f"{heading_error:.6f}",
                    f"{vel_error:.6f}",
                    f"{time_diff:.6f}",
                ]
            )

    def mapper_callback(self, msg):
        self.mapper_states.append(_extract_state(msg))

    def fg_callback(self, msg):
        fg_time, fg_x, fg_y, fg_theta, fg_v = _extract_state(msg)
        nearest_mapper = _nearest_by_time(list(self.mapper_states), fg_time)
        if nearest_mapper is None:
            return

        mapper_time, mapper_x, mapper_y, mapper_theta, mapper_v = nearest_mapper
        time_diff = abs(fg_time - mapper_time)
        if time_diff > self.max_sync_diff:
            return

        pos_error = math.hypot(fg_x - mapper_x, fg_y - mapper_y)
        heading_error = abs(self.normalize_angle(fg_theta - mapper_theta)) * 180.0 / math.pi
        vel_error = abs(fg_v - mapper_v)

        self.time_errors.append(time_diff)
        self.position_errors.append(pos_error)
        self.heading_errors.append(heading_error)
        self.velocity_errors.append(vel_error)
        self._append_csv_row(fg_time, pos_error, heading_error, vel_error, time_diff)

        if len(self.position_errors) % 100 == 0:
            self.print_stats()

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def print_stats(self):
        if not self.position_errors:
            return

        print(f"\n=== 对比统计 (共{len(self.position_errors)}帧) ===")
        print(
            "位置误差: 平均={:.3f}m, 最大={:.3f}m, 95分位={:.3f}m".format(
                np.mean(self.position_errors),
                np.max(self.position_errors),
                np.percentile(self.position_errors, 95),
            )
        )
        print(
            "航向误差: 平均={:.2f}°, 最大={:.2f}°, 95分位={:.2f}°".format(
                np.mean(self.heading_errors),
                np.max(self.heading_errors),
                np.percentile(self.heading_errors, 95),
            )
        )
        print(
            "速度误差: 平均={:.3f}m/s, 最大={:.3f}m/s".format(
                np.mean(self.velocity_errors),
                np.max(self.velocity_errors),
            )
        )
        print(f"时间差平均: {np.mean(self.time_errors) * 1000:.1f}ms")

    def generate_report(self, plot_file="", with_plot=True):
        if not self.position_errors:
            print("⚠️  没有收集到对比数据")
            return False

        print("\n" + "=" * 50)
        print("最终对比结果:")
        self.print_stats()

        if not with_plot:
            return True

        _, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))

        ax1.plot(self.position_errors)
        ax1.set_title("Position Error (m)")
        ax1.set_ylabel("Error (m)")
        ax1.grid(True)

        ax2.plot(self.heading_errors)
        ax2.set_title("Heading Error (deg)")
        ax2.set_ylabel("Error (deg)")
        ax2.grid(True)

        ax3.plot(self.velocity_errors)
        ax3.set_title("Velocity Error (m/s)")
        ax3.set_ylabel("Error (m/s)")
        ax3.set_xlabel("Frame")
        ax3.grid(True)

        ax4.hist(self.position_errors, bins=50, alpha=0.7, label="Position")
        ax4.set_title("Error Distribution")
        ax4.set_xlabel("Error")
        ax4.set_ylabel("Count")
        ax4.legend()
        ax4.grid(True)

        if not plot_file:
            plot_file = str(Path(self.output_file).with_suffix(".png"))
        plt.tight_layout()
        plt.savefig(plot_file, dpi=100)
        print(f"\n对比图表已保存到: {plot_file}")
        print(f"详细数据已保存到: {self.output_file}")
        return True


def main(argv=None):
    args = parse_args(argv)
    rospy.init_node("backend_comparer", anonymous=True)
    comparer = BackendComparer(
        output_file=args.output,
        mapper_topic=args.mapper_topic,
        fg_topic=args.fg_topic,
        max_sync_diff=args.max_sync_diff,
    )

    try:
        if args.duration > 0:
            end_time = time.time() + args.duration
            while not rospy.is_shutdown() and time.time() < end_time:
                rospy.sleep(0.05)
        else:
            rospy.spin()
    except KeyboardInterrupt:
        print("\n[!] 用户中断")

    comparer.generate_report(plot_file=args.plot_output, with_plot=not args.no_plot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
