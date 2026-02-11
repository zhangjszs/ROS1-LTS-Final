#!/bin/bash
# Non-vision mode geometry robustness verification

set -e
ERRORS=0

check_config() {
    local file=$1
    local key=$2
    local desc=$3
    
    if grep -q "$key" "$file" 2>/dev/null; then
        echo "✓ $desc"
    else
        echo "✗ $desc (missing)"
        ERRORS=$((ERRORS + 1))
    fi
}

echo "========================================"
echo "Non-Vision Geometry Robustness Check"
echo "========================================"
echo ""

echo "1. Non-Vision Mode..."
check_config "src/localization_ros/config/location_common.yaml" "w_color.*0.0" "w_color = 0.0"
check_config "src/localization_ros/config/location_common.yaml" "vision_mode:" "vision_mode config"
echo ""

echo "2. Stacked Cone Dedup..."
check_config "src/perception_ros/config/lidar_base.yaml" "stacked_cone_detection:" "stacked_cone_detection"
echo ""

echo "3. Missing Cone Fallback..."
check_config "src/localization_ros/config/location_common.yaml" "missing_cone_fallback:" "missing_cone_fallback"
check_config "src/perception_ros/config/lidar_base.yaml" "occlusion_handling:" "occlusion_handling"
echo ""

echo "4. Short Path Suppression..."
check_config "src/localization_ros/config/location_common.yaml" "short_path_suppression:" "short_path_suppression"
check_config "src/perception_ros/config/lidar_base.yaml" "short_detection_suppression:" "short_detection_suppression"
echo ""

echo "5. Loop Closure..."
check_config "src/localization_ros/config/location_common.yaml" "loop_closure:" "loop_closure"
echo ""

echo "========================================"
if [ $ERRORS -eq 0 ]; then
    echo "✓ All configs verified"
    exit 0
else
    echo "✗ $ERRORS items missing"
    exit 1
fi
