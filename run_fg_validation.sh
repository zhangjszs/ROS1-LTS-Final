#!/bin/bash
source devel/setup.bash

echo "[+] Starting FG backend validation test..."
echo "[+] Using track.bag for testing..."

# Run roslaunch in background
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=~/rosbag/track.bag \
    publish_dual_backends:=true backend:=factor_graph \
    launch_rviz:=false launch_viz:=false rate:=2.0 &
LAUNCH_PID=$!

# Wait for launch to start
sleep 15

echo "[+] Starting backend comparison tool..."
python3 perf_reports/scripts/compare_backends.py --duration 60 --output fg_validation_results.csv &
COMPARE_PID=$!

# Wait for comparison to complete
wait $COMPARE_PID

# Kill the roslaunch process
kill $LAUNCH_PID
sleep 5
pkill -f rosbag
pkill -f roslaunch

echo "[+] Validation completed! Results saved to fg_validation_results.csv"
