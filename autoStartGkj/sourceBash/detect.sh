#!/bin/bash
dataROOT="$HOME/2025huat/autoStartGkj"
${dataROOT}/sourceBash/cameraDetect.sh & ${dataROOT}/sourceBash/lidarDetect.sh & ${dataROOT}/sourceBash/imuDetect.sh
