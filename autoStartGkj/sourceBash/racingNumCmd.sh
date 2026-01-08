#!/bin/bash
cd ~/
source ~/.bashrc
echo "racingNumCmd Start Successfully!!"
#exe_op_interface="line_creat2/devel/lib/ros_vehicle_racing_num/ros_vehicle_racing_num_node"
exe_op_interface="rosrun ros_vehicle_racing_num ros_vehicle_racing_num_node"
$exe_op_interface
