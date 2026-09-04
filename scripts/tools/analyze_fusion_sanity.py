#!/usr/bin/env python3
"""Analyze fused detections to verify fusion path behavior."""

import collections
import sys

import rospy

from autodrive_msgs.msg import HUAT_FusedConeDetections

STATUS_NAMES = {
    0: "MATCHED",
    1: "NO_SYNC_PAIR",
    2: "CAMERA_INFO_MISSING",
    3: "TF_MISSING",
    4: "BEHIND_CAMERA",
    5: "OUT_OF_IMAGE",
    6: "LOW_VISION_CONFIDENCE",
    7: "NO_BBOX_MATCH",
    8: "LEGACY_HFOV_MATCH",
}

MAX_FRAMES = 50


def main():
    rospy.init_node("fusion_sanity_analyzer")

    frame_count = 0
    status_counts = collections.Counter()
    color_source_counts = collections.Counter()
    legacy_found = False

    def cb(msg):
        nonlocal frame_count, legacy_found
        if frame_count >= MAX_FRAMES:
            return

        frame_count += 1
        for s in msg.association_status:
            status_counts[s] += 1
            if s == 8:
                legacy_found = True

        # Classify color source per cone
        for i, status in enumerate(msg.association_status):
            if status == 0:
                color_source_counts["projection"] += 1
            elif status == 8:
                color_source_counts["legacy_hfov"] += 1
            else:
                # Any non-MATCHED, non-LEGACY status means LiDAR color is kept
                color_source_counts["lidar_geometry"] += 1

        if frame_count >= MAX_FRAMES:
            rospy.signal_shutdown("done")

    sub = rospy.Subscriber("/perception/fusion/detections", HUAT_FusedConeDetections, cb)
    rospy.sleep(0.5)  # wait for subscription

    timeout = rospy.Duration(30.0)
    start = rospy.Time.now()
    while frame_count < MAX_FRAMES:
        if rospy.is_shutdown():
            break
        if rospy.Time.now() - start > timeout:
            break
        try:
            rospy.sleep(0.1)
        except rospy.exceptions.ROSInterruptException:
            break

    sub.unregister()

    print(f"\n=== Fusion Sanity Report ({frame_count} frames) ===\n")
    print("Association status distribution:")
    total = sum(status_counts.values())
    for status, count in sorted(status_counts.items()):
        pct = 100.0 * count / total if total else 0
        name = STATUS_NAMES.get(status, f"UNKNOWN({status})")
        print(f"  {name:25s}: {count:6d} ({pct:5.1f}%)")

    print(f"\nColor source distribution:")
    total_cs = sum(color_source_counts.values())
    for source, count in sorted(color_source_counts.items()):
        pct = 100.0 * count / total_cs if total_cs else 0
        print(f"  {source:25s}: {count:6d} ({pct:5.1f}%)")

    print(f"\nLegacy HFOV found: {'YES (FAIL)' if legacy_found else 'NO (PASS)'}")

    if legacy_found:
        print("\n[FAIL] Legacy HFOV association status detected. Config change not effective.")
        sys.exit(1)
    elif frame_count == 0:
        print("\n[FAIL] No frames received within timeout.")
        sys.exit(1)
    else:
        print("\n[PASS] No legacy HFOV. Fusion path is working as expected.")
        sys.exit(0)


if __name__ == "__main__":
    main()
