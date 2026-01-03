#!/bin/bash
cd ~/
source ~/.bashrc
echo "racingNumCmd Start Successfully!!"
#exe_op_interface="line_creat2/devel/lib/ros_vehicle_racingNum/ros_vehicle_racingNum_node"
exe_op_interface="rosrun ros_vehicle_racingNum ros_vehicle_racingNum_node"
$exe_op_interface

