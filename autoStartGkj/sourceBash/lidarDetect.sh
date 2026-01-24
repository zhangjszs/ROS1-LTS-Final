#!/bin/bash
cd ~/
source ~/.bashrc

dataROOT="$HOME/2025huat/autoStartGkj"
var=$(cat /sys/class/net/enp2s0f1/carrier 2>/dev/null || echo 0)

while [ "$var" -eq 1 ] 2>/dev/null
do
    echo "lidar is connected"
    echo $var 
    sleep 1
    var=$(cat /sys/class/net/enp2s0f1/carrier 2>/dev/null || echo 0)
done

echo "lidar is not found!"

echo '0' > ${dataROOT}/command
