#!/bin/bash
#
# M5 Validation Script
# Automated acceptance testing for unified planning pipeline
#
# Usage:
#   ./m5_validation.sh                    # Run full validation
#   ./m5_validation.sh --ci               # CI mode (non-interactive)
#   ./m5_validation.sh --mission trackdrive --bag /path/to/bag.bag
#   ./m5_validation.sh --quick            # Quick validation (15s per mission)
#
# Output:
#   m5_report_YYYYMMDD_HHMMSS.json - Full validation report
#   m5_latest_report.json - Symlink to latest report
#   m5_logs_YYYYMMDD_HHMMSS/ - Detailed logs

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
DEFAULT_BAG_DIR="${HOME}/rosbag"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
REPORT_FILE="m5_report_${TIMESTAMP}.json"
LOG_DIR="m5_logs_${TIMESTAMP}"
CI_MODE=false
QUICK_MODE=false
SPECIFIC_MISSION=""
SPECIFIC_BAG=""
TEST_DURATION=30  # seconds per mission
QUICK_DURATION=15

# Thresholds
MIN_PATHLIMITS_HZ=8.0
MAX_PATHLIMITS_INTERVAL=0.25
MIN_CMD_HZ=10.0
MAX_CMD_INTERVAL=0.20

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --ci)
      CI_MODE=true
      shift
      ;;
    --quick)
      QUICK_MODE=true
      TEST_DURATION=$QUICK_DURATION
      shift
      ;;
    --mission)
      SPECIFIC_MISSION="$2"
      shift 2
      ;;
    --bag)
      SPECIFIC_BAG="$2"
      shift 2
      ;;
    --help|-h)
      echo "M5 Validation Script"
      echo ""
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --ci              Run in CI mode (non-interactive, exit code on failure)"
      echo "  --quick           Quick validation (15s per mission instead of 30s)"
      echo "  --mission NAME    Test specific mission only (trackdrive|acceleration|skidpad)"
      echo "  --bag PATH        Use specific bag file"
      echo "  --help            Show this help"
      echo ""
      echo "Examples:"
      echo "  $0                                    # Full validation"
      echo "  $0 --ci                               # CI mode"
      echo "  $0 --quick                            # Quick validation"
      echo "  $0 --mission trackdrive --bag ~/rosbag/track.bag"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Use --help for usage"
      exit 1
      ;;
  esac
done

# Setup
cd "$WORKSPACE_DIR"
mkdir -p "$LOG_DIR"

# Source ROS
source /opt/ros/noetic/setup.bash 2>/dev/null || true
source devel/setup.bash

# Colors for output
if [ "$CI_MODE" = false ]; then
  RED='\033[0;31m'
  GREEN='\033[0;32m'
  YELLOW='\033[1;33m'
  NC='\033[0m' # No Color
else
  RED=''
  GREEN=''
  YELLOW=''
  NC=''
fi

# Logging functions
log_info() {
  echo -e "${GREEN}[INFO]${NC} $1"
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] [INFO] $1" >> "$LOG_DIR/validation.log"
}

log_warn() {
  echo -e "${YELLOW}[WARN]${NC} $1"
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] [WARN] $1" >> "$LOG_DIR/validation.log"
}

log_error() {
  echo -e "${RED}[ERROR]${NC} $1"
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] [ERROR] $1" >> "$LOG_DIR/validation.log"
}

# Function to measure topic metrics
measure_topic() {
  local topic=$1
  local duration=$2
  local output_file=$3
  
  timeout $duration rostopic hz "$topic" 2>&1 > "$output_file" &
  local pid=$!
  wait $pid 2>/dev/null || true
  
  # Parse results
  local avg_hz=$(grep "average rate:" "$output_file" | tail -1 | awk '{print $3}')
  local min_interval=$(grep "min:" "$output_file" | tail -1 | awk '{print $2}')
  local max_interval=$(grep "max:" "$output_file" | tail -1 | awk '{print $4}')
  
  # Default to 0 if parsing failed
  avg_hz=${avg_hz:-0}
  max_interval=${max_interval:-999}
  
  echo "{\"hz\": $avg_hz, \"max_interval\": $max_interval}"
}

# Function to check log for errors
check_log_errors() {
  local log_file=$1
  local curvature_exceeds=0
  local failsafe_count=0
  
  if [ -f "$log_file" ]; then
    curvature_exceeds=$(grep -c "Curvature exceeds" "$log_file" 2>/dev/null || echo 0)
    failsafe_count=$(grep -c "GENERAL FAILSAFE" "$log_file" 2>/dev/null || echo 0)
  fi
  
  echo "{\"curvature_exceeds\": $curvature_exceeds, \"general_failsafe\": $failsafe_count}"
}

# Function to run single mission test
run_mission_test() {
  local mission=$1
  local bag_path=$2
  local duration=${3:-$TEST_DURATION}
  
  log_info "Testing mission: $mission"
  log_info "Bag file: $bag_path"
  log_info "Duration: ${duration}s"
  
  local mission_log="$LOG_DIR/${mission}.log"
  local pathlimits_hz_log="$LOG_DIR/${mission}_pathlimits_hz.log"
  local cmd_hz_log="$LOG_DIR/${mission}_cmd_hz.log"
  
  # Start launch in background
  case $mission in
    trackdrive)
      roslaunch fsd_launch trackdrive.launch \
        simulation:=true \
        bag:="$bag_path" \
        launch_rviz:=false \
        launch_viz:=false \
        > "$mission_log" 2>&1 &
      ;;
    acceleration)
      roslaunch fsd_launch acceleration.launch \
        simulation:=true \
        bag:="$bag_path" \
        launch_rviz:=false \
        launch_viz:=false \
        > "$mission_log" 2>&1 &
      ;;
    skidpad)
      roslaunch fsd_launch skidpad.launch \
        simulation:=true \
        bag:="$bag_path" \
        launch_rviz:=false \
        launch_viz:=false \
        > "$mission_log" 2>&1 &
      ;;
    *)
      log_error "Unknown mission: $mission"
      return 1
      ;;
  esac
  
  local launch_pid=$!
  
  # Wait for system to stabilize
  log_info "Waiting for system stabilization (15s)..."
  sleep 15
  
  # Check if still running
  if ! kill -0 $launch_pid 2>/dev/null; then
    log_error "Launch failed or exited early. Check log: $mission_log"
    return 1
  fi
  
  # Measure topics
  log_info "Measuring planning/pathlimits..."
  local pathlimits_metrics=$(measure_topic "/planning/pathlimits" $duration "$pathlimits_hz_log")
  
  log_info "Measuring vehcileCMDMsg..."
  local cmd_metrics=$(measure_topic "/vehcileCMDMsg" $duration "$cmd_hz_log")
  
  # Stop launch
  log_info "Stopping launch..."
  kill $launch_pid 2>/dev/null || true
  sleep 3
  
  # Parse metrics
  local pl_hz=$(echo "$pathlimits_metrics" | python3 -c "import sys,json; print(json.load(sys.stdin)['hz'])")
  local pl_interval=$(echo "$pathlimits_metrics" | python3 -c "import sys,json; print(json.load(sys.stdin)['max_interval'])")
  local cmd_hz=$(echo "$cmd_metrics" | python3 -c "import sys,json; print(json.load(sys.stdin)['hz'])")
  local cmd_interval=$(echo "$cmd_metrics" | python3 -c "import sys,json; print(json.load(sys.stdin)['max_interval'])")
  
  # Check for errors
  local error_metrics=$(check_log_errors "$mission_log")
  local curvature_exceeds=$(echo "$error_metrics" | python3 -c "import sys,json; print(json.load(sys.stdin)['curvature_exceeds'])")
  local failsafe_count=$(echo "$error_metrics" | python3 -c "import sys,json; print(json.load(sys.stdin)['general_failsafe'])")
  
  # Determine pass/fail
  local pass=true
  
  if (( $(echo "$pl_hz < $MIN_PATHLIMITS_HZ" | bc -l) )); then
    log_warn "$mission: PathLimits frequency $pl_hz Hz < $MIN_PATHLIMITS_HZ Hz"
    pass=false
  fi
  
  if (( $(echo "$pl_interval > $MAX_PATHLIMITS_INTERVAL" | bc -l) )); then
    log_warn "$mission: PathLimits max interval ${pl_interval}s > ${MAX_PATHLIMITS_INTERVAL}s"
    pass=false
  fi
  
  if (( $(echo "$cmd_hz < $MIN_CMD_HZ" | bc -l) )); then
    log_warn "$mission: Cmd frequency $cmd_hz Hz < $MIN_CMD_HZ Hz"
    pass=false
  fi
  
  if (( $(echo "$cmd_interval > $MAX_CMD_INTERVAL" | bc -l) )); then
    log_warn "$mission: Cmd max interval ${cmd_interval}s > ${MAX_CMD_INTERVAL}s"
    pass=false
  fi
  
  if [ "$pass" = true ]; then
    log_info "$mission: ✅ PASSED"
  else
    log_warn "$mission: ❌ FAILED"
  fi
  
  # Output JSON result
  cat <<EOF
{
  "mission": "$mission",
  "pathlimits_hz": $pl_hz,
  "pathlimits_max_interval": $pl_interval,
  "cmd_hz": $cmd_hz,
  "cmd_max_interval": $cmd_interval,
  "curvature_exceeds": $curvature_exceeds,
  "general_failsafe": $failsafe_count,
  "pass": $pass
}
EOF
}

# Main validation
log_info "========================================"
log_info "M5 Unified Planner Validation"
log_info "========================================"
log_info "Timestamp: $TIMESTAMP"
log_info "Workspace: $WORKSPACE_DIR"
log_info "Mode: $([ "$CI_MODE" = true ] && echo "CI" || echo "Interactive")"
log_info "Duration: $([ "$QUICK_MODE" = true ] && echo "Quick ($TEST_DURATION"s")" || echo "Full ($TEST_DURATION"s")")"
log_info ""

# Get git info
GIT_COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")
GIT_TAG=$(git describe --tags --exact-match 2>/dev/null || echo "none")
log_info "Git commit: $GIT_COMMIT"
log_info "Git tag: $GIT_TAG"

# Check dependencies
if ! command -v python3 &> /dev/null; then
  log_error "python3 not found"
  exit 1
fi

if ! python3 -c "import json" 2>/dev/null; then
  log_error "python3 json module not available"
  exit 1
fi

# Initialize report
REPORT_JSON="{
  \"timestamp\": \"$(date -Iseconds)\",
  \"baseline_tag\": \"m5-baseline\",
  \"commit\": \"$GIT_COMMIT\",
  \"git_tag\": \"$GIT_TAG\",
  \"validator_version\": \"1.0\",
  \"mode\": \"$([ "$CI_MODE" = true ] && echo "ci" || ([ "$QUICK_MODE" = true ] && echo "quick" || echo "full"))\",
  \"results\": {},
  \"overall_pass\": true,
  \"should_rollback\": false
}"

# Run tests
OVERALL_PASS=true

if [ -n "$SPECIFIC_MISSION" ]; then
  # Test specific mission
  if [ -z "$SPECIFIC_BAG" ]; then
    SPECIFIC_BAG="${DEFAULT_BAG_DIR}/${SPECIFIC_MISSION}.bag"
    [ "$SPECIFIC_MISSION" = "trackdrive" ] && SPECIFIC_BAG="${DEFAULT_BAG_DIR}/track.bag"
    [ "$SPECIFIC_MISSION" = "acceleration" ] && SPECIFIC_BAG="${DEFAULT_BAG_DIR}/accel.bag"
  fi
  
  if [ ! -f "$SPECIFIC_BAG" ]; then
    log_error "Bag file not found: $SPECIFIC_BAG"
    exit 1
  fi
  
  RESULT=$(run_mission_test "$SPECIFIC_MISSION" "$SPECIFIC_BAG" $TEST_DURATION)
  REPORT_JSON=$(echo "$REPORT_JSON" | python3 -c "
import sys, json
data = json.load(sys.stdin)
result = json.loads('$RESULT')
data['results']['$SPECIFIC_MISSION'] = result
if not result['pass']:
  data['overall_pass'] = False
print(json.dumps(data, indent=2))
")
  
  OVERALL_PASS=$(echo "$REPORT_JSON" | python3 -c "import sys,json; print(json.load(sys.stdin)['overall_pass'])")
else
  # Test all missions
  declare -A MISSION_BAGS=(
    ["trackdrive"]="${DEFAULT_BAG_DIR}/track.bag"
    ["acceleration"]="${DEFAULT_BAG_DIR}/accel.bag"
    ["skidpad"]="${DEFAULT_BAG_DIR}/skidpad.bag"
  )
  
  for mission in trackdrive acceleration skidpad; do
    bag="${MISSION_BAGS[$mission]}"
    
    if [ ! -f "$bag" ]; then
      log_warn "Bag file not found: $bag, skipping $mission"
      REPORT_JSON=$(echo "$REPORT_JSON" | python3 -c "
import sys, json
data = json.load(sys.stdin)
data['results']['$mission'] = {'skipped': True, 'reason': 'Bag not found'}
print(json.dumps(data, indent=2))
")
      continue
    fi
    
    RESULT=$(run_mission_test "$mission" "$bag" $TEST_DURATION)
    REPORT_JSON=$(echo "$REPORT_JSON" | python3 -c "
import sys, json
data = json.load(sys.stdin)
result = json.loads('$RESULT')
data['results']['$mission'] = result
if not result['pass']:
  data['overall_pass'] = False
print(json.dumps(data, indent=2))
")
  done
  
  OVERALL_PASS=$(echo "$REPORT_JSON" | python3 -c "import sys,json; print(json.load(sys.stdin)['overall_pass'])")
fi

# Finalize report
REPORT_JSON=$(echo "$REPORT_JSON" | python3 -c "
import sys, json
data = json.load(sys.stdin)
data['overall_pass'] = $OVERALL_PASS
data['should_rollback'] = not data['overall_pass']
print(json.dumps(data, indent=2))
")

# Save report
echo "$REPORT_JSON" > "$REPORT_FILE"
ln -sf "$REPORT_FILE" m5_latest_report.json

log_info ""
log_info "========================================"
if [ "$OVERALL_PASS" = true ]; then
  log_info "✅ VALIDATION PASSED"
else
  log_error "❌ VALIDATION FAILED"
fi
log_info "========================================"
log_info "Report: $REPORT_FILE"
log_info "Logs: $LOG_DIR/"

# CI mode exit code
if [ "$CI_MODE" = true ]; then
  [ "$OVERALL_PASS" = true ] && exit 0 || exit 1
fi

exit 0
