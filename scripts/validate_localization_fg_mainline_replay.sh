#!/usr/bin/env bash
# Backward-compatibility forwarder -> scripts/validation/validate_localization_fg_mainline_replay.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/validation/validate_localization_fg_mainline_replay.sh" "$@"
