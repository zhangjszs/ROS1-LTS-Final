#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULT_DIR="${RESULT_DIR:-/tmp/b123_validation}"
BAG_PATH="${1:-/home/kerwin/rosbag/track.bag}"
PLAY_RATE="${2:-2.0}"
TIMEOUT_S="${TIMEOUT_S:-95}"

mkdir -p "${RESULT_DIR}" /tmp/roslog
rm -f "${RESULT_DIR}/launch.log" \
      "${RESULT_DIR}/record.log" \
      "${RESULT_DIR}/contract_source.bag" \
      "${RESULT_DIR}/contract_report.txt"

source "${ROOT_DIR}/devel/setup.bash"
export ROS_LOG_DIR=/tmp/roslog

cleanup() {
  if [[ -n "${REC_PID:-}" ]]; then
    kill "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[B123] Start one replay validation"
echo "[B123] bag=${BAG_PATH} rate=${PLAY_RATE} result_dir=${RESULT_DIR}"

timeout "${TIMEOUT_S}s" roslaunch fsd_launch trackdrive.launch \
  simulation:=true \
  bag:="${BAG_PATH}" \
  rate:="${PLAY_RATE}" \
  loop:=false \
  launch_rviz:=false \
  launch_viz:=false \
  > "${RESULT_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 30); do
  if rostopic list >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

rosbag record -O "${RESULT_DIR}/contract_source.bag" \
  /planning/pathlimits \
  /localization/cone_map \
  /perception/lidar_cluster/detections \
  > "${RESULT_DIR}/record.log" 2>&1 &
REC_PID=$!

wait "${LAUNCH_PID}" || true
unset LAUNCH_PID
sleep 2
kill "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" 2>/dev/null || true
unset REC_PID

python3 - "${RESULT_DIR}/contract_source.bag" > "${RESULT_DIR}/contract_report.txt" <<'PY'
import math
import sys

import rosbag

bag_path = sys.argv[1]

failures = []
stats = {
    "pathlimits_count": 0,
    "cone_map_count": 0,
    "detections_count": 0,
}

def to_sec(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9

with rosbag.Bag(bag_path, "r") as bag:
    for topic, msg, _ in bag.read_messages():
        if topic == "/planning/pathlimits":
            stats["pathlimits_count"] += 1
            if msg.header.frame_id != "world":
                failures.append(f"pathlimits.frame_id={msg.header.frame_id}")
            if msg.header.stamp.secs == 0 and msg.header.stamp.nsecs == 0:
                failures.append("pathlimits.header.stamp is zero")
            if msg.stamp.secs == 0 and msg.stamp.nsecs == 0:
                failures.append("pathlimits.stamp is zero")
            if to_sec(msg.stamp) < to_sec(msg.header.stamp):
                failures.append("pathlimits.stamp < header.stamp")
            n = len(msg.path)
            if len(msg.target_speeds) != n or len(msg.curvatures) != n:
                failures.append(
                    f"pathlimits.shape mismatch path={n}, speed={len(msg.target_speeds)}, curv={len(msg.curvatures)}"
                )

        elif topic == "/localization/cone_map":
            stats["cone_map_count"] += 1
            if msg.header.frame_id != "world":
                failures.append(f"cone_map.frame_id={msg.header.frame_id}")
            for cone in msg.cone:
                if cone.confidence < 0 or cone.confidence > 1000:
                    failures.append(f"cone_map.confidence out of range: {cone.confidence}")
                    break

        elif topic == "/perception/lidar_cluster/detections":
            stats["detections_count"] += 1
            if msg.header.frame_id != "velodyne":
                failures.append(f"detections.frame_id={msg.header.frame_id}")
            for conf in msg.confidence:
                if (not math.isfinite(conf)) or conf < 0.0 or conf > 1.0:
                    failures.append(f"detections.confidence out of range: {conf}")
                    break

print("[B123] message counts:", stats)
if stats["pathlimits_count"] == 0:
    failures.append("no /planning/pathlimits samples")
if stats["cone_map_count"] == 0:
    failures.append("no /localization/cone_map samples")
if stats["detections_count"] == 0:
    failures.append("no /perception/lidar_cluster/detections samples")

if failures:
    print("[B123] FAIL")
    for item in failures:
        print(" -", item)
    sys.exit(1)

print("[B123] PASS")
PY

cat "${RESULT_DIR}/contract_report.txt"
echo "[B123] artifacts:"
echo "  - ${RESULT_DIR}/launch.log"
echo "  - ${RESULT_DIR}/record.log"
echo "  - ${RESULT_DIR}/contract_source.bag"
echo "  - ${RESULT_DIR}/contract_report.txt"
