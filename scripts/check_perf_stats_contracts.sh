#!/usr/bin/env bash
# Backward-compatibility forwarder -> scripts/ci/check_perf_stats_contracts.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/ci/check_perf_stats_contracts.sh" "$@"
