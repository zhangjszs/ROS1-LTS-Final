#!/bin/sh

# 快速性能报告生成脚本
# 用于在rosbag测试后快速生成性能报告

NOTE=""
SCENARIO=""
TAGS=""

usage() {
    echo "用法: $0 [--note \"变更说明\"] [--scenario \"场景\"] [--tags tag1,tag2]"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --note)
            if [ -z "${2-}" ]; then
                echo "缺少参数: --note"
                usage
                exit 1
            fi
            NOTE="$2"
            shift 2
            ;;
        --scenario)
            if [ -z "${2-}" ]; then
                echo "缺少参数: --scenario"
                usage
                exit 1
            fi
            SCENARIO="$2"
            shift 2
            ;;
        --tags)
            if [ -z "${2-}" ]; then
                echo "缺少参数: --tags"
                usage
                exit 1
            fi
            TAGS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            usage
            exit 1
            ;;
    esac
done

echo "=========================================="
echo "性能报告快速生成工具"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

# 使用统一的 perf_tool 运行完整工作流
set --
if [ -n "${NOTE}" ]; then
    set -- "$@" --note "${NOTE}"
fi
if [ -n "${SCENARIO}" ]; then
    set -- "$@" --scenario "${SCENARIO}"
fi
if [ -n "${TAGS}" ]; then
    set -- "$@" --tags "${TAGS}"
fi

python3 scripts/perf_tool.py full "$@"

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✅ 所有步骤完成！"
    echo "=========================================="
    echo ""
    echo "生成的文件："
    ls -lh reports/ 2>/dev/null | tail -n +2 | head -5
else
    echo ""
    echo "❌ 执行失败"
    exit 1
fi
