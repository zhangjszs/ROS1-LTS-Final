#!/usr/bin/env bash
# Backward-compatibility forwarder -> scripts/ci/check_perception_regression_mode.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/ci/check_perception_regression_mode.sh" "$@"
