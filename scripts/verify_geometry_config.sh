#!/bin/bash
# Non-vision mode geometry robustness verification
# Three-layer check: YAML key existence, YAML value correctness, C++ loading code

set -e
ERRORS=0

check_config() {
    local file=$1
    local key=$2
    local desc=$3

    if grep -q "$key" "$file" 2>/dev/null; then
        echo "  ✓ $desc"
    else
        echo "  ✗ $desc (missing)"
        ERRORS=$((ERRORS + 1))
    fi
}

check_value() {
    local file=$1
    local pattern=$2
    local desc=$3

    if grep -qE "$pattern" "$file" 2>/dev/null; then
        echo "  ✓ $desc"
    else
        echo "  ✗ $desc (value mismatch)"
        ERRORS=$((ERRORS + 1))
    fi
}

check_cpp_load() {
    local file=$1
    local pattern=$2
    local desc=$3

    if grep -q "$pattern" "$file" 2>/dev/null; then
        echo "  ✓ C++ loads: $desc"
    else
        echo "  ✗ C++ missing: $desc"
        ERRORS=$((ERRORS + 1))
    fi
}

echo "========================================"
echo "Non-Vision Geometry Robustness Check"
echo "========================================"

echo ""
echo "--- Layer 1: YAML Key Existence ---"
echo ""

echo "1. Non-Vision Mode..."
check_config "src/localization_ros/config/location_common.yaml" "w_color.*0.0" "w_color = 0.0"
check_config "src/localization_ros/config/location_common.yaml" "vision_mode:" "vision_mode config"
echo ""

echo "2. Stacked Cone Detection..."
check_config "src/perception_ros/config/lidar_base.yaml" "stacked_cone_detection:" "stacked_cone_detection"
echo ""

echo "3. Missing Cone Fallback..."
check_config "src/localization_ros/config/location_common.yaml" "missing_cone_fallback:" "missing_cone_fallback"
echo ""

echo "4. Short Path Suppression..."
check_config "src/localization_ros/config/location_common.yaml" "short_path_suppression:" "short_path_suppression"
echo ""

echo "5. Loop Closure..."
check_config "src/localization_ros/config/location_common.yaml" "loop_closure:" "loop_closure"
echo ""

echo "--- Layer 2: YAML Value Correctness ---"
echo ""

check_value "src/localization_ros/config/location_common.yaml" "w_color:.*0\.0" "w_color is 0.0"
check_value "src/localization_ros/config/location_common.yaml" "enabled:.*false" "vision_mode disabled"
check_value "src/localization_ros/config/location_common.yaml" "expected_spacing:.*5\.0" "expected_spacing = 5.0"
check_value "src/localization_ros/config/location_common.yaml" "min_path_length:.*3\.0" "min_path_length = 3.0"
check_value "src/perception_ros/config/lidar_base.yaml" "layer_height:.*0\.25" "layer_height = 0.25"
echo ""

echo "--- Layer 3: C++ Loading Code ---"
echo ""

check_cpp_load "src/localization_ros/src/location.cpp" "missing_cone_fallback/enabled" "missing_cone_fallback params"
check_cpp_load "src/localization_ros/src/location.cpp" "short_path_suppression/enabled" "short_path_suppression params"
check_cpp_load "src/perception_ros/src/lidar_cluster_ros.cpp" "stacked_cone_detection/enabled" "stacked_cone_detection params"
check_cpp_load "src/localization_core/src/location_mapper.cpp" "interpolateMissingCones" "interpolateMissingCones impl"
check_cpp_load "src/localization_core/src/location_mapper.cpp" "short_path_suppression" "short_path_suppression impl"
check_cpp_load "src/perception_core/src/utility.cpp" "stacked_enable" "stacked cone Z-axis check"
echo ""

echo "--- Layer 4: Runtime Check (optional) ---"
echo ""

if command -v rosparam &>/dev/null && rosparam list &>/dev/null 2>&1; then
    echo "  roscore detected, checking runtime params..."
    for key in missing_cone_fallback/enabled short_path_suppression/enabled; do
        if rosparam get "/$key" &>/dev/null 2>&1; then
            val=$(rosparam get "/$key")
            echo "  ✓ rosparam /$key = $val"
        else
            echo "  - rosparam /$key not loaded (node not running)"
        fi
    done
else
    echo "  (skipped - roscore not running)"
fi
echo ""

echo "========================================"
if [ $ERRORS -eq 0 ]; then
    echo "✓ All checks passed ($ERRORS errors)"
    exit 0
else
    echo "✗ $ERRORS items failed"
    exit 1
fi
