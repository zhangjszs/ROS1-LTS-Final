#!/bin/bash
set -e
cd /home/kerwin/2025huat
source devel/setup.bash
export PYTHONPATH="/home/kerwin/2025huat/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH}"

roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/home/kerwin/rosbag/track.bag loop:=true launch_rviz:=false launch_viz:=false &
ROSLAUNCH_PID=$!
sleep 8
echo "Roslaunch started, running benchmark..."
python3 scripts/benchmark_trackdrive.py --output /tmp/benchmark_sota.json --duration 45
echo "Benchmark done, killing roslaunch..."
kill $ROSLAUNCH_PID 2>/dev/null || true
wait $ROSLAUNCH_PID 2>/dev/null || true
echo "Done"
