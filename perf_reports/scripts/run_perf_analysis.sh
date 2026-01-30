#!/bin/bash

# 性能分析脚本
# 用于自动化性能数据收集、报告生成和分析

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PERF_REPORTS_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== 性能分析工作流 ==="
echo ""

# 使用统一的 perf_tool 运行完整工作流
python3 "$SCRIPT_DIR/perf_tool.py" full "$@"

echo ""
echo "=== 完成 ==="
echo "所有报告已生成到 $PERF_REPORTS_DIR/reports/ 目录"
