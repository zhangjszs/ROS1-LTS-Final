#!/bin/bash
# Fusion Sanity Test for track.bag with new fusion/enabled config.
#
# Purpose: Verify that when camera_info is missing, legacy HFOV does NOT run
# and cones fallback to LiDAR geometry color (CAMERA_INFO_MISSING status).
#
# Usage:
#   cd ~/2025huat
#   bash scripts/run_fusion_sanity_test.sh [path/to/track.bag]
#
# Prerequisites:
#   - Workspace built: catkin build
#   - track.bag available (default: ~/rosbag/track.bag)
#   - ROS Noetic environment

set -o pipefail

# Config
WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/2025huat}"
BAG_PATH="${1:-$HOME/rosbag/track.bag}"
LAUNCH_SLEEP_SEC="${LAUNCH_SLEEP_SEC:-12}"
VISION_SLEEP_SEC="${VISION_SLEEP_SEC:-10}"

cd "$WORKSPACE_DIR" || { echo "ERROR: Cannot cd to $WORKSPACE_DIR"; exit 1; }

if [ ! -f "$BAG_PATH" ]; then
    echo "ERROR: Bag not found: $BAG_PATH"
    echo "Usage: $0 [path/to/track.bag]"
    exit 1
fi

source devel/setup.bash
export PYTHONPATH="$WORKSPACE_DIR/devel/lib/python3/dist-packages:$PYTHONPATH"

echo "=== Fusion Sanity Test ==="
echo "Bag: $BAG_PATH"
echo "Workspace: $WORKSPACE_DIR"
echo ""

# Cleanup helper
cleanup() {
    echo "=== Cleaning up ==="
    pkill -f "fake_vision_publisher.py" 2>/dev/null || true
    if [ -n "$LAUNCH_PID" ]; then
        kill "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# Launch lidar cluster + rosbag play
echo "=== Starting replay ==="
roslaunch perception_ros lidar_cluster.launch \
    bag:="$BAG_PATH" \
    simulation:=true \
    mode:=track \
    launch_rviz:=false \
    > /tmp/fusion_sanity_launch.log 2>&1 &
LAUNCH_PID=$!

sleep "$LAUNCH_SLEEP_SEC"

# Start fake vision publisher (syncs to point cloud timestamps)
echo "=== Starting fake vision publisher ==="
python3 "$WORKSPACE_DIR/scripts/fake_vision_publisher.py" \
    > /tmp/fusion_sanity_vision.log 2>&1 &
VISION_PID=$!

sleep "$VISION_SLEEP_SEC"

# Run analyzer
echo "=== Running analyzer ==="
python3 "$WORKSPACE_DIR/scripts/analyze_fusion_sanity.py"
ANALYZER_EXIT=$?

if [ "$ANALYZER_EXIT" -ne 0 ]; then
    echo ""
    echo "=== SANITY CHECK FAILED ==="
    echo "See logs:"
    echo "  Launch:  /tmp/fusion_sanity_launch.log"
    echo "  Vision:  /tmp/fusion_sanity_vision.log"
    exit 1
fi

echo ""
echo "=== SANITY CHECK PASSED ==="
exit 0
