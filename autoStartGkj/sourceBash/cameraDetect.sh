#!/bin/bash
cd ~/
source ~/.bashrc

cameraPath="/home/tb/autoStart/start.sh"
var=$(cat /sys/class/net/enp2s0f3/carrier)
while [ $var -eq 1 ]
do
    echo "camera is connected"
    sleep 1
    var=$(cat /sys/class/net/enp2s0f3/carrier)
done

echo "camera is not found!"

echo '0' > /home/tb/autoStartGkj/command
