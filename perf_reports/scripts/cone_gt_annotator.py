#!/usr/bin/env python3
"""Generate a GT cone CSV template from replay detections."""

import argparse
import csv
import math
from pathlib import Path


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="从 bag 检测输出生成 GT CSV 初稿")
    parser.add_argument("--bag", required=True, help="输入 rosbag 路径")
    parser.add_argument("--output", required=True, help="输出 GT CSV 路径")
    parser.add_argument(
        "--topic",
        default="/perception/lidar_cluster/detections",
        help="锥桶检测话题",
    )
    parser.add_argument(
        "--max-messages",
        type=int,
        default=300,
        help="最多读取的消息帧数",
    )
    parser.add_argument(
        "--merge-radius",
        type=float,
        default=0.6,
        help="聚类半径（米）",
    )
    parser.add_argument(
        "--min-observations",
        type=int,
        default=3,
        help="最小观测次数，低于该值的点会被丢弃",
    )
    return parser.parse_args(argv)


def _distance_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def cluster_points(points, merge_radius):
    clusters = []
    for x, y, z in points:
        nearest_idx = -1
        nearest_dist = float("inf")
        for idx, cluster in enumerate(clusters):
            dist = _distance_xy((x, y), (cluster["x"], cluster["y"]))
            if dist < merge_radius and dist < nearest_dist:
                nearest_idx = idx
                nearest_dist = dist

        if nearest_idx < 0:
            clusters.append({"x": x, "y": y, "z": z, "count": 1})
            continue

        cluster = clusters[nearest_idx]
        count = cluster["count"] + 1
        cluster["x"] = (cluster["x"] * cluster["count"] + x) / count
        cluster["y"] = (cluster["y"] * cluster["count"] + y) / count
        cluster["z"] = (cluster["z"] * cluster["count"] + z) / count
        cluster["count"] = count

    return clusters


def load_detection_points_from_bag(bag_path, topic, max_messages):
    try:
        import rosbag
    except ImportError as exc:  # pragma: no cover - runtime dependency
        raise RuntimeError("rosbag 未安装或未 source ROS 环境") from exc

    all_points = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        frame_count = 0
        for _, msg, _ in bag.read_messages(topics=[topic]):
            frame_count += 1
            for point in msg.points:
                all_points.append((float(point.x), float(point.y), float(point.z)))
            if max_messages > 0 and frame_count >= max_messages:
                break
    return all_points


def write_gt_csv(output_path, clusters, min_observations):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    valid = [c for c in clusters if c["count"] >= min_observations]
    with output_path.open("w", newline="", encoding="utf-8") as file_obj:
        file_obj.write("# Auto-generated GT cone map template\n")
        file_obj.write("# format: x,y,z\n")
        writer = csv.writer(file_obj)
        for cluster in valid:
            writer.writerow(
                [
                    f"{cluster['x']:.3f}",
                    f"{cluster['y']:.3f}",
                    f"{cluster['z']:.3f}",
                ]
            )
    return len(valid)


def main(argv=None):
    args = parse_args(argv)
    points = load_detection_points_from_bag(
        bag_path=args.bag,
        topic=args.topic,
        max_messages=max(0, args.max_messages),
    )
    if not points:
        print("[!] 未读取到任何检测点，未生成 GT 文件")
        return 1

    clusters = cluster_points(points, merge_radius=max(0.01, args.merge_radius))
    written = write_gt_csv(args.output, clusters, min_observations=max(1, args.min_observations))

    print("[+] 输入点数:", len(points))
    print("[+] 聚类数:", len(clusters))
    print("[+] 输出 GT 点数:", written)
    print("[+] GT CSV:", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
