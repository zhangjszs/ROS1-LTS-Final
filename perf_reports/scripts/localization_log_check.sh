#!/bin/bash
# localization_log_check.sh - 定位日志分析脚本
# 分析rosout日志，查找定位模块的错误、警告和异常

set -euo pipefail

LOG_FILE="${1:-}"
SEARCH_PATTERN="${2:-localization}"

if [ -z "$LOG_FILE" ] || [ ! -f "$LOG_FILE" ]; then
    echo "用法: $0 <rosout.log文件路径> [搜索关键词，默认localization]"
    echo "示例: $0 ~/.ros/log/latest/rosout.log"
    exit 1
fi

echo "=== 定位日志分析 ==="
echo "日志文件: $LOG_FILE"
echo "搜索关键词: $SEARCH_PATTERN"
echo "分析时间: $(date)"
echo ""

# 统计错误
echo "--- 错误统计 ---"
error_count=$(grep -c "\[ERROR\].*$SEARCH_PATTERN" "$LOG_FILE" || true)
echo "错误数量: $error_count"
if [ "$error_count" -gt 0 ]; then
    echo ""
    grep "\[ERROR\].*$SEARCH_PATTERN" "$LOG_FILE" | tail -10
    echo ""
fi

# 统计警告
echo "--- 警告统计 ---"
warn_count=$(grep -c "\[WARN\].*$SEARCH_PATTERN" "$LOG_FILE" || true)
echo "警告数量: $warn_count"
if [ "$warn_count" -gt 0 ]; then
    echo ""
    grep "\[WARN\].*$SEARCH_PATTERN" "$LOG_FILE" | tail -10
    echo ""
fi

# 统计重定位事件
echo "--- 重定位事件 ---"
reloc_count=$(grep -c "reloc.*success\|relocalization.*success\|Relocation.*success" "$LOG_FILE" || true)
echo "重定位成功次数: $reloc_count"

reloc_fail_count=$(grep -c "reloc.*fail\|relocalization.*fail\|Relocation.*fail" "$LOG_FILE" || true)
echo "重定位失败次数: $reloc_fail_count"
echo ""

# 检查定位状态变化
echo "--- 定位状态变化 ---"
grep "TRACKING\|DEGRADED\|INS_ONLY" "$LOG_FILE" | tail -10
echo ""

# 检查时间跳变
echo "--- 异常时间跳变 ---"
grep "time jump\|clock jump\|time went backwards" "$LOG_FILE" || echo "无异常时间跳变"
echo ""

echo "分析完成！"
echo "总错误: $error_count, 总警告: $warn_count, 重定位成功率: $(if [ $((reloc_count + reloc_fail_count)) -gt 0 ]; then echo "scale=2; $reloc_count / ($reloc_count + $reloc_fail_count) * 100" | bc; else echo "N/A"; fi)%"