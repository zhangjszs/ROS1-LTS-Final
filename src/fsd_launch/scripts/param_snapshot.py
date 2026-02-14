#!/usr/bin/env python3
"""
Parameter Snapshot Tool
启动时自动 dump 所有 rosparam 到文件，支持 diff 两次运行的参数差异。

Usage:
  As a ROS node (auto-snapshot at startup):
    rosrun fsd_launch param_snapshot.py

  As a standalone diff tool:
    python3 param_snapshot.py --diff <snap1.yaml> <snap2.yaml>
"""

import os
import sys
import argparse
from datetime import datetime

import yaml


def get_snapshot_dir():
    """Return ~/.fsd/param_snapshots/, creating if needed."""
    d = os.path.expanduser("~/.fsd/param_snapshots")
    os.makedirs(d, exist_ok=True)
    return d


def take_snapshot():
    """Dump all rosparam to a timestamped YAML file. Returns the file path."""
    import rospy

    rospy.init_node("param_snapshot", anonymous=True)
    rospy.sleep(2.0)  # wait for params to settle

    params = rospy.get_param("/")
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(get_snapshot_dir(), f"{ts}.yaml")

    with open(path, "w") as f:
        yaml.dump(params, f, default_flow_style=False, allow_unicode=True)

    rospy.loginfo(f"[param_snapshot] Saved to {path}")
    return path


def flatten(d, prefix=""):
    """Flatten a nested dict into {'/a/b/c': value} form."""
    items = {}
    for k, v in d.items():
        key = f"{prefix}/{k}" if prefix else k
        if isinstance(v, dict):
            items.update(flatten(v, key))
        else:
            items[key] = v
    return items


def diff_snapshots(path_a, path_b):
    """Print parameter differences between two snapshot files."""
    with open(path_a) as f:
        a = yaml.safe_load(f) or {}
    with open(path_b) as f:
        b = yaml.safe_load(f) or {}

    fa, fb = flatten(a), flatten(b)
    all_keys = sorted(set(fa) | set(fb))

    added, removed, changed = [], [], []
    for k in all_keys:
        if k not in fa:
            added.append((k, fb[k]))
        elif k not in fb:
            removed.append((k, fa[k]))
        elif fa[k] != fb[k]:
            changed.append((k, fa[k], fb[k]))

    if not added and not removed and not changed:
        print("No differences.")
        return

    if added:
        print(f"\n--- Added ({len(added)}) ---")
        for k, v in added:
            print(f"  + {k}: {v}")
    if removed:
        print(f"\n--- Removed ({len(removed)}) ---")
        for k, v in removed:
            print(f"  - {k}: {v}")
    if changed:
        print(f"\n--- Changed ({len(changed)}) ---")
        for k, old, new in changed:
            print(f"  ~ {k}: {old} -> {new}")

    print(f"\nTotal: +{len(added)} -{len(removed)} ~{len(changed)}")


def main():
    parser = argparse.ArgumentParser(description="ROS Parameter Snapshot Tool")
    parser.add_argument("--diff", nargs=2, metavar=("SNAP1", "SNAP2"),
                        help="Diff two snapshot files")
    parser.add_argument("--list", action="store_true",
                        help="List existing snapshots")
    args = parser.parse_args()

    if args.diff:
        diff_snapshots(args.diff[0], args.diff[1])
    elif args.list:
        d = get_snapshot_dir()
        files = sorted(f for f in os.listdir(d) if f.endswith(".yaml"))
        if not files:
            print("No snapshots found.")
        for f in files:
            path = os.path.join(d, f)
            size = os.path.getsize(path)
            print(f"  {f}  ({size} bytes)")
    else:
        take_snapshot()


if __name__ == "__main__":
    main()
