#!/usr/bin/env bash
# Backward-compatibility forwarder -> scripts/ci/check_deprecation_contracts.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/ci/check_deprecation_contracts.sh" "$@"
