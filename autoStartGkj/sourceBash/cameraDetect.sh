#!/bin/bash
cd ~/
source ~/.bashrc

dataROOT="$HOME/2025huat/autoStartGkj"
var=$(cat /sys/class/net/enp2s0f3/carrier 2>/dev/null || echo 0)
while [ "$var" -eq 1 ] 2>/dev/null
do
    echo "camera is connected"
    sleep 1
    var=$(cat /sys/class/net/enp2s0f3/carrier 2>/dev/null || echo 0)
done

echo "camera is not found!"

echo '0' > ${dataROOT}/command
