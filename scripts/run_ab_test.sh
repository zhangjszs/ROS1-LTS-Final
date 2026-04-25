#!/bin/bash
set -e

VARIANT="$1"
OUTPUT="$2"
DURATION="${3:-60}"
BAG="${4:-/home/kerwin/rosbag/track.bag}"

if [ -z "$VARIANT" ] || [ -z "$OUTPUT" ]; then
    echo "Usage: $0 <variant_name> <output_json> [duration_sec] [bag_path]"
    exit 1
fi

echo "[A/B Test] Starting $VARIANT benchmark..."
echo "  Output: $OUTPUT"
echo "  Duration: ${DURATION}s"
echo "  Bag: $BAG"

cd /home/kerwin/2025huat
source /home/kerwin/2025huat/devel/setup.bash

# Verify fsd_launch is findable
if ! rospack find fsd_launch > /dev/null 2>&1; then
    echo "[A/B Test] ERROR: fsd_launch package not found. Did you build and source?"
    exit 1
fi

# Kill any existing ros nodes
pkill -f "rosrun\|roslaunch\|roscore" 2>/dev/null || true
sleep 2

# Start roscore in background
roscore &
ROSCORE_PID=$!
sleep 3

# Launch trackdrive in simulation mode (no rviz to save resources)
roslaunch fsd_launch trackdrive.launch simulation:=true bag:="$BAG" launch_rviz:=false launch_viz:=false rate:=1.0 &
LAUNCH_PID=$!

# Wait for system to stabilize - fixed 25s wait for all nodes to initialize and start publishing
echo "[A/B Test] Waiting 25s for system stabilization..."
sleep 25

# Quick topic check
echo "[A/B Test] Checking topics..."
rostopic list | grep -E "perception/lidar_cluster/detections|planning/pathlimits|vehicle/cmd|localization/car_state" || true

# Run benchmark
python3 /home/kerwin/2025huat/scripts/benchmark_trackdrive.py --duration "$DURATION" --output "$OUTPUT" &
BENCH_PID=$!

wait $BENCH_PID
BENCH_EXIT=$?

# Cleanup
kill $LAUNCH_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true
pkill -f "rosrun\|roslaunch\|roscore" 2>/dev/null || true
sleep 2

if [ $BENCH_EXIT -eq 0 ]; then
    echo "[A/B Test] $VARIANT benchmark completed successfully."
    echo "  Result: $OUTPUT"
else
    echo "[A/B Test] $VARIANT benchmark failed with exit code $BENCH_EXIT"
    exit 1
fi
