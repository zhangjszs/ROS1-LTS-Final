#!/usr/bin/env bash
set -euo pipefail
WS="${1:-$(pwd)}"
grep -RIn -E "control reaches end of non-void function|-Wreturn-type|Package name \".*\" does not follow" "$WS/logs" || true
