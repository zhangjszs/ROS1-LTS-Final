#!/bin/sh

# 性能报告快速启动脚本
# 用于在rosbag测试后快速生成性能报告

NOTE=""
SCENARIO=""
TAGS=""
RUN_COMPARE=0
RUN_TIMELINE=0

usage() {
    echo "用法: $0 [--note \"变更说明\"] [--scenario \"场景\"] [--tags tag1,tag2] [--compare] [--timeline]"
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
        --compare)
            RUN_COMPARE=1
            shift
            ;;
        --timeline)
            RUN_TIMELINE=1
            shift
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
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PERF_DIR="${PROJECT_DIR}/perf_reports"
SCRIPTS_DIR="${PERF_DIR}/scripts"

cd "${PROJECT_DIR}" || exit 1

echo "1. 收集性能数据..."
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
python3 "${SCRIPTS_DIR}/collect_perf_data.py" "$@"

if [ $? -eq 0 ]; then
    echo "✅ 数据收集完成"
else
    echo "❌ 数据收集失败"
    exit 1
fi

echo ""
echo "2. 生成性能报告..."
python3 "${SCRIPTS_DIR}/generate_report.py"

if [ $? -eq 0 ]; then
    echo "✅ 报告生成完成"
else
    echo "❌ 报告生成失败"
    exit 1
fi

echo ""
echo "3. 生成性能图表..."
python3 "${SCRIPTS_DIR}/generate_charts.py"

if [ $? -eq 0 ]; then
    echo "✅ 图表生成完成"
else
    echo "❌ 图表生成失败"
    exit 1
fi

if [ ${RUN_COMPARE} -eq 1 ]; then
    echo ""
    echo "4. 生成对比报告..."
    python3 "${SCRIPTS_DIR}/generate_report.py" --compare
    if [ $? -eq 0 ]; then
        echo "✅ 对比报告生成完成"
    else
        echo "❌ 对比报告生成失败"
        exit 1
    fi
    echo ""
    echo "5. 生成对比图表..."
    python3 "${SCRIPTS_DIR}/generate_charts.py" --compare
    if [ $? -eq 0 ]; then
        echo "✅ 对比图表生成完成"
    else
        echo "❌ 对比图表生成失败"
        exit 1
    fi
fi

if [ ${RUN_TIMELINE} -eq 1 ]; then
    echo ""
    echo "6. 生成时间线报告..."
    python3 "${SCRIPTS_DIR}/generate_report.py" --timeline
    if [ $? -eq 0 ]; then
        echo "✅ 时间线报告生成完成"
    else
        echo "❌ 时间线报告生成失败"
        exit 1
    fi
fi

echo ""
echo "=========================================="
echo "✅ 所有步骤完成！"
echo "=========================================="
echo ""
echo "生成的文件："
ls -lh "${PERF_DIR}/reports/" | tail -n +2
echo ""
echo "查看报告："
echo "  cat ${PERF_DIR}/reports/perf_report_*.md"
if [ ${RUN_COMPARE} -eq 1 ]; then
    echo "  cat ${PERF_DIR}/reports/perf_comparison_*.md"
fi
if [ ${RUN_TIMELINE} -eq 1 ]; then
    echo "  cat ${PERF_DIR}/reports/perf_timeline_*.md"
fi
echo ""
echo "查看图表："
echo "  eog ${PERF_DIR}/reports/*.png"
echo ""
