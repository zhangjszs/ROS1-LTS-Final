#!/bin/bash
cd ~/
source ~/.bashrc

imuPath="/dev/ttyUSB0"

while [ -e "$imuPath" ]
do
    echo "imu is connected"
    sleep 1
done

echo "imu is not found!"

echo '0' > /home/tb/autoStartGkj/command
