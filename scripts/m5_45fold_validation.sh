#!/bin/bash
#
# M5 45-Fold Fixed Coverage Validation
# 固定覆盖验证：9 bag files × 5 repeats = 45 runs
#
# Usage:
#   ./m5_45fold_validation.sh
#
# Requirements:
#   - 9 bag files in ~/rosbag/ (3 per mission)
#   - Baseline metrics from m5-baseline tag

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
REPORT_DIR="m5_45fold_reports_${TIMESTAMP}"
LOG_FILE="${REPORT_DIR}/validation.log"

# Fixed coverage config
REPEATS=5
TEST_DURATION=30

# M5 Baseline metrics (hard thresholds)
BASELINE_TRACKDRIVE_PL_HZ=9.97
BASELINE_TRACKDRIVE_PL_MAX=0.153
BASELINE_TRACKDRIVE_CMD_HZ=10.0
BASELINE_TRACKDRIVE_CMD_MAX=0.106

BASELINE_ACCEL_PL_HZ=19.99
BASELINE_ACCEL_PL_MAX=0.053
BASELINE_ACCEL_CMD_HZ=9.997
BASELINE_ACCEL_CMD_MAX=0.104

BASELINE_SKIDPAD_PL_HZ=20.01
BASELINE_SKIDPAD_PL_MAX=0.056
BASELINE_SKIDPAD_CMD_HZ=10.0
BASELINE_SKIDPAD_CMD_MAX=0.107

# Source ROS
source /opt/ros/noetic/setup.bash 2>/dev/null || true
source "${WORKSPACE_DIR}/devel/setup.bash"

cd "$WORKSPACE_DIR"
mkdir -p "$REPORT_DIR"

echo "========================================" | tee -a "$LOG_FILE"
echo "M5 45-Fold Fixed Coverage Validation" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "Timestamp: $TIMESTAMP" | tee -a "$LOG_FILE"
echo "Repeats: $REPEATS per bag" | tee -a "$LOG_FILE"
echo "Total runs: $((9 * REPEATS))" | tee -a "$LOG_FILE"
echo "Baseline: m5-baseline" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Track all results
ALL_PASSED=true
TOTAL_RUNS=0
PASS_COUNT=0

# Function to run single validation
run_single_validation() {
    local mission=$1
    local bag=$2
    local run_num=$3
    
    local bag_name=$(basename "$bag")
    echo "[$mission] Run $run_num: $bag_name" | tee -a "$LOG_FILE"
    
    # Run validation
    local result
    if ! result=$("${SCRIPT_DIR}/m5_validation.sh" --mission "$mission" --bag "$bag" --ci 2>/dev/null); then
        echo "  FAILED: Validation script error" | tee -a "$LOG_FILE"
        return 1
    fi
    
    # Extract metrics
    local pl_hz=$(echo "$result" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d['results']['$mission']['pathlimits_hz'])")
    local pl_max=$(echo "$result" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d['results']['$mission']['pathlimits_max_interval'])")
    local cmd_hz=$(echo "$result" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d['results']['$mission']['cmd_hz'])")
    local cmd_max=$(echo "$result" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d['results']['$mission']['cmd_max_interval'])")
    local curvature=$(echo "$result" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d['results']['$mission'].get('curvature_exceeds', 0))")
    local failsafe=$(echo "$result" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d['results']['$mission'].get('general_failsafe', 0))")
    
    # Check against baseline
    local passed=true
    
    case $mission in
        trackdrive)
            if (( $(echo "$pl_hz < $BASELINE_TRACKDRIVE_PL_HZ" | bc -l) )); then
                echo "  FAIL: pathlimits_hz $pl_hz < baseline $BASELINE_TRACKDRIVE_PL_HZ" | tee -a "$LOG_FILE"
                passed=false
            fi
            if (( $(echo "$pl_max > $BASELINE_TRACKDRIVE_PL_MAX" | bc -l) )); then
                echo "  FAIL: pathlimits_max $pl_max > baseline $BASELINE_TRACKDRIVE_PL_MAX" | tee -a "$LOG_FILE"
                passed=false
            fi
            if (( $(echo "$cmd_hz < $BASELINE_TRACKDRIVE_CMD_HZ" | bc -l) )); then
                echo "  FAIL: cmd_hz $cmd_hz < baseline $BASELINE_TRACKDRIVE_CMD_HZ" | tee -a "$LOG_FILE"
                passed=false
            fi
            if (( $(echo "$cmd_max > $BASELINE_TRACKDRIVE_CMD_MAX" | bc -l) )); then
                echo "  FAIL: cmd_max $cmd_max > baseline $BASELINE_TRACKDRIVE_CMD_MAX" | tee -a "$LOG_FILE"
                passed=false
            fi
            ;;
        acceleration)
            if (( $(echo "$pl_hz < $BASELINE_ACCEL_PL_HZ" | bc -l) )); then
                echo "  FAIL: pathlimits_hz $pl_hz < baseline $BASELINE_ACCEL_PL_HZ" | tee -a "$LOG_FILE"
                passed=false
            fi
            if (( $(echo "$pl_max > $BASELINE_ACCEL_PL_MAX" | bc -l) )); then
                echo "  FAIL: pathlimits_max $pl_max > baseline $BASELINE_ACCEL_PL_MAX" | tee -a "$LOG_FILE"
                passed=false
            fi
            if (( $(echo "$cmd_hz < $BASELINE_ACCEL_CMD_HZ" | bc -l) )); then
                echo "  FAIL: cmd_hz $cmd_hz < baseline $BASELINE_ACCEL_CMD_HZ" | tee -a "$LOG_FILE"
                passed=false
            fi
            if (( $(echo "$cmd_max > $BASELINE_ACCEL_CMD_MAX" | bc -l) )); then
                echo "  FAIL: cmd_max $cmd_max > baseline $BASELINE_ACCEL_CMD_MAX" | tee -a "$LOG_FILE"
                passed=false
            fi
            ;;
        skidpad)
            if (( $(echo "$pl_hz < $BASELINE_SKIDPAD_PL_HZ" | bc -l) )); then
                echo "  FAIL: pathlimits_hz $pl_hz < baseline $BASELINE_SKIDPAD_PL_HZ" | tee -a "$LOG_FILE"
                passed=false
            fi
            if (( $(echo "$pl_max > $BASELINE_SKIDPAD_PL_MAX" | bc -l) )); then
                echo "  FAIL: pathlimits_max $pl_max > baseline $BASELINE_SKIDPAD_PL_MAX" | tee -a "$LOG_FILE"
                passed=false
            fi
            if (( $(echo "$cmd_hz < $BASELINE_SKIDPAD_CMD_HZ" | bc -l) )); then
                echo "  FAIL: cmd_hz $cmd_hz < baseline $BASELINE_SKIDPAD_CMD_HZ" | tee -a "$LOG_FILE"
                passed=false
            fi
            if (( $(echo "$cmd_max > $BASELINE_SKIDPAD_CMD_MAX" | bc -l) )); then
                echo "  FAIL: cmd_max $cmd_max > baseline $BASELINE_SKIDPAD_CMD_MAX" | tee -a "$LOG_FILE"
                passed=false
            fi
            ;;
    esac
    
    # Safety checks
    if [ "$failsafe" -ne 0 ]; then
        echo "  FAIL: GENERAL FAILSAFE = $failsafe (must be 0)" | tee -a "$LOG_FILE"
        passed=false
    fi
    
    if [ "$passed" = true ]; then
        echo "  ✓ PASSED" | tee -a "$LOG_FILE"
        return 0
    else
        echo "  ✗ FAILED" | tee -a "$LOG_FILE"
        return 1
    fi
}

# Find all bags
BAG_DIR="${HOME}/rosbag"
declare -A BAGS
declare -A MISSIONS

# Trackdrive bags
BAGS[0]="${BAG_DIR}/track.bag"
MISSIONS[0]="trackdrive"

# Acceleration bags  
BAGS[1]="${BAG_DIR}/accel.bag"
MISSIONS[1]="acceleration"

# Skidpad bags
BAGS[2]="${BAG_DIR}/skidpad.bag"
MISSIONS[2]="skidpad"

# Check which bags exist
VALID_BAGS=()
VALID_MISSIONS=()
for i in 0 1 2; do
    if [ -f "${BAGS[$i]}" ]; then
        VALID_BAGS+=("${BAGS[$i]}")
        VALID_MISSIONS+=("${MISSIONS[$i]}")
        echo "Found bag: ${MISSIONS[$i]} -> ${BAGS[$i]}" | tee -a "$LOG_FILE"
    else
        echo "WARNING: Bag not found: ${BAGS[$i]}" | tee -a "$LOG_FILE"
    fi
done

if [ ${#VALID_BAGS[@]} -eq 0 ]; then
    echo "ERROR: No valid bags found!" | tee -a "$LOG_FILE"
    exit 1
fi

echo "" | tee -a "$LOG_FILE"
echo "Starting validation with ${#VALID_BAGS[@]} bags × $REPEATS repeats..." | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Run all validations
for ((repeat=1; repeat<=REPEATS; repeat++)); do
    echo "========================================" | tee -a "$LOG_FILE"
    echo "REPEAT $repeat / $REPEATS" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
    
    for ((i=0; i<${#VALID_BAGS[@]}; i++)); do
        mission="${VALID_MISSIONS[$i]}"
        bag="${VALID_BAGS[$i]}"
        
        TOTAL_RUNS=$((TOTAL_RUNS + 1))
        
        if run_single_validation "$mission" "$bag" "$repeat"; then
            PASS_COUNT=$((PASS_COUNT + 1))
        else
            ALL_PASSED=false
        fi
        
        # Small delay between runs
        sleep 2
    done
done

# Generate final report
echo "" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "45-FOLD VALIDATION COMPLETE" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "Total runs: $TOTAL_RUNS" | tee -a "$LOG_FILE"
echo "Passed: $PASS_COUNT" | tee -a "$LOG_FILE"
echo "Failed: $((TOTAL_RUNS - PASS_COUNT))" | tee -a "$LOG_FILE"
echo "Success rate: $(echo "scale=2; $PASS_COUNT * 100 / $TOTAL_RUNS" | bc)%" | tee -a "$LOG_FILE"

if [ "$ALL_PASSED" = true ]; then
    echo "" | tee -a "$LOG_FILE"
    echo "✅ ALL $TOTAL_RUNS RUNS PASSED" | tee -a "$LOG_FILE"
    echo "Ready for Phase 3 (legacy cleanup)" | tee -a "$LOG_FILE"
    echo "" | tee -a "$LOG_FILE"
    echo "⚠️  NOTE: Validation based on $TOTAL_RUNS bag runs only." | tee -a "$LOG_FILE"
    echo "   Real vehicle / new scenarios risks not covered." | tee -a "$LOG_FILE"
    exit 0
else
    echo "" | tee -a "$LOG_FILE"
    echo "❌ VALIDATION FAILED" | tee -a "$LOG_FILE"
    echo "Fix issues before Phase 3" | tee -a "$LOG_FILE"
    exit 1
fi
