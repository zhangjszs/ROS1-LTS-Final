#!/bin/bash
#
# Emergency Rollback Script for Unified Planner
# Quickly revert to legacy planners if unified planner fails
#
# Usage:
#   ./emergency_rollback.sh [MISSION]
#
# Examples:
#   ./emergency_rollback.sh              # Auto-detect mission
#   ./emergency_rollback.sh trackdrive   # Rollback trackdrive
#   ./emergency_rollback.sh acceleration # Rollback acceleration
#   ./emergency_rollback.sh skidpad      # Rollback skidpad

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() {
  echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
  echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
  echo -e "${RED}[ERROR]${NC} $1"
}

# Determine mission
MISSION="${1:-}"

if [ -z "$MISSION" ]; then
  # Try to auto-detect from running ROS parameters
  log_info "Auto-detecting mission from ROS parameters..."
  MISSION=$(rosparam get /planning_pipeline/mission 2>/dev/null || echo "")
  
  if [ -z "$MISSION" ]; then
    log_error "Could not auto-detect mission"
    log_info "Please specify mission: $0 [trackdrive|acceleration|skidpad]"
    exit 1
  fi
  
  log_info "Detected mission: $MISSION"
fi

# Map mission to legacy planner
case $MISSION in
  trackdrive|autocross|high_speed)
    LEGACY_PLANNER="high_speed"
    LAUNCH_FILE="trackdrive"
    ;;
  acceleration|line)
    LEGACY_PLANNER="line"
    LAUNCH_FILE="acceleration"
    ;;
  skidpad)
    LEGACY_PLANNER="skidpad"
    LAUNCH_FILE="skidpad"
    ;;
  *)
    log_error "Unknown mission: $MISSION"
    log_info "Supported missions: trackdrive, acceleration, skidpad"
    exit 1
    ;;
esac

# Log rollback event
LOG_FILE="/tmp/planning_rollback.log"
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Rolling back mission=$MISSION to legacy=$LEGACY_PLANNER" >> "$LOG_FILE"

log_warn "========================================"
log_warn "EMERGENCY ROLLBACK INITIATED"
log_warn "========================================"
log_info "Mission: $MISSION"
log_info "Legacy planner: $LEGACY_PLANNER"
log_info "Log file: $LOG_FILE"

# Check if ROS is running
if ! rostopic list > /dev/null 2>&1; then
  log_error "ROS master not running!"
  log_info "Cannot perform rollback - ROS is not active"
  exit 1
fi

# Stop current planning nodes
log_info "Stopping current planning nodes..."
rosnode kill /planning_pipeline 2>/dev/null || true
rosnode kill /high_speed_tracking 2>/dev/null || true
rosnode kill /line_detection 2>/dev/null || true
rosnode kill /skidpad_detection 2>/dev/null || true

sleep 2

# Verify nodes are stopped
REMAINING=$(rosnode list | grep -E "(planning_pipeline|high_speed_tracking|line_detection|skidpad_detection)" || true)
if [ -n "$REMAINING" ]; then
  log_warn "Some nodes still running:"
  echo "$REMAINING"
  log_info "Force killing..."
  echo "$REMAINING" | while read node; do
    rosnode kill "$node" 2>/dev/null || true
  done
fi

# Source workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_DIR"
source devel/setup.bash

# Start legacy planner
log_info "Starting legacy planner: $LEGACY_PLANNER"
log_info "Launch file: ${LAUNCH_FILE}.launch"

# Note: In real emergency, you'd need to specify bag or connect to live sensors
# This script just shows the command - actual restart needs context
log_warn ""
log_warn "To complete rollback, run:"
log_warn ""
log_warn "  roslaunch fsd_launch ${LAUNCH_FILE}.launch planner:=$LEGACY_PLANNER"
log_warn ""
log_info "Or with a bag file:"
log_info ""
log_info "  roslaunch fsd_launch ${LAUNCH_FILE}.launch planner:=$LEGACY_PLANNER simulation:=true bag:=/path/to/bag.bag"
log_info ""

# Write rollback command to file for easy copy-paste
ROLLBACK_CMD="roslaunch fsd_launch ${LAUNCH_FILE}.launch planner:=$LEGACY_PLANNER"
echo "$ROLLBACK_CMD" > /tmp/last_rollback_command.txt

log_info "Rollback command saved to: /tmp/last_rollback_command.txt"

# Verify unified planner is stopped
sleep 1
if rosnode list | grep -q "/planning_pipeline"; then
  log_error "WARNING: Unified planner still running!"
  log_info "Try: rosnode kill /planning_pipeline"
else
  log_info "✅ Unified planner stopped"
fi

log_warn ""
log_warn "========================================"
log_warn "ROLLBACK READY - EXECUTE COMMAND ABOVE"
log_warn "========================================"

# Show quick verification commands
log_info ""
log_info "After starting legacy planner, verify with:"
log_info "  rostopic hz /planning/pathlimits"
log_info "  rostopic hz /vehcileCMDMsg"
log_info ""

exit 0
