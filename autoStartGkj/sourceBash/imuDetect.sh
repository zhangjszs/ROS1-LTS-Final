#!/bin/bash
cd ~/
source ~/.bashrc

dataROOT="$HOME/2025huat/autoStartGkj"
imuPath="/dev/ttyUSB0"

while [ -e "$imuPath" ]
do
    echo "imu is connected"
    sleep 1
done

echo "imu is not found!"

echo '0' > ${dataROOT}/command
