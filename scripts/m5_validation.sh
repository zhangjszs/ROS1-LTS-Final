#!/usr/bin/env bash
# Backward-compatibility forwarder -> scripts/validation/m5_validation.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/validation/m5_validation.sh" "$@"
