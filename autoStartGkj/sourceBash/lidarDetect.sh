#!/bin/bash
cd ~/
source ~/.bashrc

lidarPath="/home/tb/autoStartGkj/start.sh"
var=$(cat /sys/class/net/enp2s0f1/carrier)

while [ $var -eq 1 ]
do
    echo "lidar is connected"
    echo $var 
    sleep 1
    var=$(cat /sys/class/net/enp2s0f1/carrier)
done

echo "lidar is not found!"

echo '0' > /home/tb/autoStartGkj/command
