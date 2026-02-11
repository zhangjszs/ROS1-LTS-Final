#!/bin/bash
#
# M5 Final Validation (3 runs - one per mission)
# Quick validation against M5 baseline before Phase 3 cleanup
#
# Usage: ./m5_final_validation.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
REPORT_DIR="m5_final_validation_${TIMESTAMP}"
mkdir -p "$REPORT_DIR"

# M5 Baseline thresholds (must meet or exceed)
declare -A BASELINE_PL_HZ BASELINE_PL_MAX BASELINE_CMD_HZ BASELINE_CMD_MAX
BASELINE_PL_HZ[trackdrive]=9.97
BASELINE_PL_MAX[trackdrive]=0.153
BASELINE_CMD_HZ[trackdrive]=10.0
BASELINE_CMD_MAX[trackdrive]=0.106

BASELINE_PL_HZ[acceleration]=19.99
BASELINE_PL_MAX[acceleration]=0.053
BASELINE_CMD_HZ[acceleration]=9.997
BASELINE_CMD_MAX[acceleration]=0.104

BASELINE_PL_HZ[skidpad]=20.01
BASELINE_PL_MAX[skidpad]=0.056
BASELINE_CMD_HZ[skidpad]=10.0
BASELINE_CMD_MAX[skidpad]=0.107

# Bag files
declare -A BAG_FILES
BAG_FILES[trackdrive]="${HOME}/rosbag/track.bag"
BAG_FILES[acceleration]="${HOME}/rosbag/accel.bag"
BAG_FILES[skidpad]="${HOME}/rosbag/skidpad.bag"

echo "========================================"
echo "M5 Final Validation (3 runs)"
echo "========================================"
echo "Timestamp: $TIMESTAMP"
echo ""

cd "$WORKSPACE_DIR"
source devel/setup.bash

ALL_PASSED=true
RESULTS_FILE="$REPORT_DIR/results.json"
echo "{" > "$RESULTS_FILE"
FIRST=true

for mission in trackdrive acceleration skidpad; do
    bag="${BAG_FILES[$mission]}"
    
    if [ ! -f "$bag" ]; then
        echo "❌ $mission: Bag not found - $bag"
        ALL_PASSED=false
        continue
    fi
    
    echo "Testing $mission..."
    
    # Run validation
    if ! result=$("${SCRIPT_DIR}/m5_validation.sh" --mission "$mission" --bag "$bag" --ci 2>/dev/null); then
        echo "  ❌ FAILED: Validation error"
        ALL_PASSED=false
        continue
    fi
    
    # Extract metrics
    pl_hz=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin)['results']['$mission']['pathlimits_hz'])")
    pl_max=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin)['results']['$mission']['pathlimits_max_interval'])")
    cmd_hz=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin)['results']['$mission']['cmd_hz'])")
    cmd_max=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin)['results']['$mission']['cmd_max_interval'])")
    failsafe=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin)['results']['$mission'].get('general_failsafe', 0))")
    
    # Check against baseline
    passed=true
    
    if (( $(echo "$pl_hz < ${BASELINE_PL_HZ[$mission]}" | bc -l) )); then
        echo "  ❌ pathlimits_hz: $pl_hz < baseline ${BASELINE_PL_HZ[$mission]}"
        passed=false
    else
        echo "  ✓ pathlimits_hz: $pl_hz >= ${BASELINE_PL_HZ[$mission]}"
    fi
    
    if (( $(echo "$pl_max > ${BASELINE_PL_MAX[$mission]}" | bc -l) )); then
        echo "  ❌ pathlimits_max: ${pl_max}s > baseline ${BASELINE_PL_MAX[$mission]}s"
        passed=false
    else
        echo "  ✓ pathlimits_max: ${pl_max}s <= ${BASELINE_PL_MAX[$mission]}s"
    fi
    
    if (( $(echo "$cmd_hz < ${BASELINE_CMD_HZ[$mission]}" | bc -l) )); then
        echo "  ❌ cmd_hz: $cmd_hz < baseline ${BASELINE_CMD_HZ[$mission]}"
        passed=false
    else
        echo "  ✓ cmd_hz: $cmd_hz >= ${BASELINE_CMD_HZ[$mission]}"
    fi
    
    if (( $(echo "$cmd_max > ${BASELINE_CMD_MAX[$mission]}" | bc -l) )); then
        echo "  ❌ cmd_max: ${cmd_max}s > baseline ${BASELINE_CMD_MAX[$mission]}s"
        passed=false
    else
        echo "  ✓ cmd_max: ${cmd_max}s <= ${BASELINE_CMD_MAX[$mission]}s"
    fi
    
    if [ "$failsafe" -ne 0 ]; then
        echo "  ❌ GENERAL FAILSAFE: $failsafe (must be 0)"
        passed=false
    else
        echo "  ✓ GENERAL FAILSAFE: 0"
    fi
    
    if [ "$passed" = true ]; then
        echo "  ✅ $mission PASSED"
    else
        echo "  ❌ $mission FAILED"
        ALL_PASSED=false
    fi
    echo ""
    
    # Save to JSON
    if [ "$FIRST" = false ]; then
        echo "," >> "$RESULTS_FILE"
    fi
    FIRST=false
    cat >> "$RESULTS_FILE" << EOF
  "$mission": {
    "pathlimits_hz": $pl_hz,
    "pathlimits_max_interval": $pl_max,
    "cmd_hz": $cmd_hz,
    "cmd_max_interval": $cmd_max,
    "general_failsafe": $failsafe,
    "passed": $passed
  }
EOF
done

echo "}" >> "$RESULTS_FILE"

echo "========================================"
if [ "$ALL_PASSED" = true ]; then
    echo "✅ ALL 3 MISSIONS PASSED"
    echo ""
    echo "Ready for Phase 3: Legacy code cleanup"
    echo ""
    echo "⚠️  RISK ACKNOWLEDGMENT:"
    echo "   Validation based on 3 bag files only."
    echo "   Real vehicle / new scenario risks not covered."
    echo ""
    echo "Report: $RESULTS_FILE"
    exit 0
else
    echo "❌ VALIDATION FAILED"
    echo "Fix issues before Phase 3"
    exit 1
fi
