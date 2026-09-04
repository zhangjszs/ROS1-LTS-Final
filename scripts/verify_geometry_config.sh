#!/usr/bin/env bash
# Backward-compatibility forwarder -> scripts/ci/verify_geometry_config.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/ci/verify_geometry_config.sh" "$@"
