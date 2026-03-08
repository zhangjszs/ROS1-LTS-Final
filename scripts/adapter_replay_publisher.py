#!/usr/bin/env python3
import argparse
import copy
import time
from typing import Callable, Dict, List, Tuple

import rospy
from autodrive_msgs.msg import HUAT_ConeDetections, HUAT_FusedConeDetections
from geometry_msgs.msg import Point32


Event = Tuple[float, str, object]


def make_point(x: float, y: float, z: float = 0.0) -> Point32:
    point = Point32()
    point.x = x
    point.y = y
    point.z = z
    return point


def make_raw(stamp: rospy.Time, colors: List[int]) -> HUAT_ConeDetections:
    msg = HUAT_ConeDetections()
    msg.header.stamp = stamp
    msg.header.frame_id = "velodyne"
    msg.color_types = list(colors)
    for i, color in enumerate(colors):
        msg.points.append(make_point(float(i + 1), float(i)))
        msg.maxPoints.append(make_point(float(i + 1), float(i), 0.4))
        msg.minPoints.append(make_point(float(i + 1), float(i), 0.0))
        msg.confidence.append(0.8)
        msg.obj_dist.append(float(i + 2))
        msg.track_ids.append(i)
    return msg


def make_fused(stamp: rospy.Time, colors: List[int]) -> HUAT_FusedConeDetections:
    msg = HUAT_FusedConeDetections()
    msg.header.stamp = stamp
    msg.header.frame_id = "velodyne"
    msg.lidar_frame = "velodyne"
    msg.lidar_stamp = stamp
    msg.vision_stamp = stamp
    msg.lidar_color_types = [4 for _ in colors]
    msg.fused_color_types = list(colors)
    msg.association_status = [0 for _ in colors]
    for i, color in enumerate(colors):
        msg.points.append(make_point(float(i + 1), float(i)))
        msg.obj_dist.append(float(i + 2))
        msg.lidar_confidences.append(0.8)
        msg.track_ids.append(i)
        msg.vision_confidences.append(900)
        msg.association_scores.append(1.0)
        msg.association_reasons.append("MATCHED")
    msg.matched_count = len(colors)
    msg.unmatched_count = 0
    return msg


def add_frame_pair(events: List[Event], stamp: rospy.Time, raw_offset: float, fused_offset: float = None,
                   raw_colors: List[int] = None, fused_colors: List[int] = None) -> None:
    raw_colors = raw_colors or [2, 3]
    fused_colors = fused_colors or [0, 1]
    events.append((raw_offset, "raw", make_raw(stamp, raw_colors)))
    if fused_offset is not None:
        events.append((fused_offset, "fused", make_fused(stamp, fused_colors)))


def scenario_raw_only() -> List[Event]:
    events: List[Event] = []
    start_stamp = rospy.Time.from_sec(1000.0)
    for i in range(40):
        stamp = start_stamp + rospy.Duration.from_sec(i * 0.1)
        add_frame_pair(events, stamp, i * 0.1, None)
    return events


def scenario_same_stamp_normal_merge() -> List[Event]:
    events: List[Event] = []
    start_stamp = rospy.Time.from_sec(1100.0)
    for i in range(40):
        base = i * 0.1
        stamp = start_stamp + rospy.Duration.from_sec(base)
        add_frame_pair(events, stamp, base, base + 0.01)
    return events


def scenario_late_fused_within_budget() -> List[Event]:
    events: List[Event] = []
    start_stamp = rospy.Time.from_sec(1200.0)
    for i in range(40):
        base = i * 0.1
        stamp = start_stamp + rospy.Duration.from_sec(base)
        add_frame_pair(events, stamp, base, base + 0.15)
    return events


def scenario_late_fused_over_deadline() -> List[Event]:
    events: List[Event] = []
    start_stamp = rospy.Time.from_sec(1300.0)
    for i in range(40):
        base = i * 0.1
        stamp = start_stamp + rospy.Duration.from_sec(base)
        add_frame_pair(events, stamp, base, base + 0.25)
    return events


def scenario_intermittent_drop_count_mismatch() -> List[Event]:
    events: List[Event] = []
    start_stamp = rospy.Time.from_sec(1400.0)
    for i in range(48):
        base = i * 0.1
        stamp = start_stamp + rospy.Duration.from_sec(base)
        pattern = i % 4
        if pattern == 0:
            add_frame_pair(events, stamp, base, base + 0.02)
        elif pattern == 1:
            add_frame_pair(events, stamp, base, None)
        elif pattern == 2:
            add_frame_pair(events, stamp, base, base + 0.02, fused_colors=[0])
        else:
            add_frame_pair(events, stamp, base, base + 0.12)
    return events


def scenario_duplicate_stale_finalize() -> List[Event]:
    events: List[Event] = []
    start_stamp = rospy.Time.from_sec(1500.0)
    for i in range(36):
        base = i * 0.1
        stamp = start_stamp + rospy.Duration.from_sec(base)
        add_frame_pair(events, stamp, base, base + 0.02 if i % 2 == 0 else None)
        if i % 2 == 0:
            events.append((base + 0.35, "raw", make_raw(stamp, [2, 3])))
            events.append((base + 0.36, "fused", make_fused(stamp, [0, 1])))
        else:
            events.append((base + 0.35, "fused", make_fused(stamp, [0, 1])))
        stale_stamp = start_stamp + rospy.Duration.from_sec(100.0 + base)
        events.append((base + 0.04, "fused", make_fused(stale_stamp, [0, 1])))
    return events


def scenario_callback_timer_pressure() -> List[Event]:
    events: List[Event] = []
    start_stamp = rospy.Time.from_sec(1600.0)
    frame_period = 0.05
    for i in range(80):
        base = i * frame_period
        stamp = start_stamp + rospy.Duration.from_sec(base)
        pattern = i % 5
        if pattern in (0, 1, 2):
            add_frame_pair(events, stamp, base, base + 0.19)
        elif pattern == 3:
            add_frame_pair(events, stamp, base, None)
        else:
            add_frame_pair(events, stamp, base, base + 0.18, fused_colors=[0])
        if i % 10 == 0:
            duplicate_stamp = copy.deepcopy(stamp)
            events.append((base + 0.28, "fused", make_fused(duplicate_stamp, [0, 1])))
    return events


SCENARIOS: Dict[str, Callable[[], List[Event]]] = {
    "raw_only": scenario_raw_only,
    "same_stamp_normal_merge": scenario_same_stamp_normal_merge,
    "late_fused_within_budget": scenario_late_fused_within_budget,
    "late_fused_over_deadline": scenario_late_fused_over_deadline,
    "intermittent_drop_count_mismatch": scenario_intermittent_drop_count_mismatch,
    "duplicate_stale_finalize": scenario_duplicate_stale_finalize,
    "callback_timer_pressure": scenario_callback_timer_pressure,
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish synthetic adapter replay scenarios at wall-clock rate.")
    parser.add_argument("--scenario", required=True, choices=sorted(SCENARIOS.keys()))
    parser.add_argument("--start-delay", type=float, default=1.0)
    args = parser.parse_args()

    rospy.init_node("adapter_replay_publisher", anonymous=True)
    raw_pub = rospy.Publisher("perception/lidar_cluster/detections", HUAT_ConeDetections, queue_size=50)
    fused_pub = rospy.Publisher("perception/fusion/detections", HUAT_FusedConeDetections, queue_size=50)

    deadline = time.time() + 10.0
    while (raw_pub.get_num_connections() == 0 or fused_pub.get_num_connections() == 0) and time.time() < deadline:
        rospy.sleep(0.05)

    events = sorted(SCENARIOS[args.scenario](), key=lambda item: item[0])
    start_time = time.monotonic() + args.start_delay

    for offset_sec, topic, message in events:
        while not rospy.is_shutdown() and time.monotonic() < start_time + offset_sec:
            time.sleep(0.001)
        if rospy.is_shutdown():
            return
        if topic == "raw":
            raw_pub.publish(message)
        else:
            fused_pub.publish(message)

    rospy.sleep(0.6)


if __name__ == "__main__":
    main()
