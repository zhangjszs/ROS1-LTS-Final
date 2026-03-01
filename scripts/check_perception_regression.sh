#!/bin/bash
#
# check_perception_regression.sh - Perception bag 回放回归测试
#
# 功能:
#   1. 回放指定 bag 文件
#   2. 录制感知输出
#   3. 计算指标并与 baseline 对比
#   4. 超阈值则返回非零状态码
#
# 用法:
#   ./check_perception_regression.sh <bag_file> [--baseline <baseline_json>] [--threshold <file>]
#
# 环境变量:
#   REGRESSION_THRESHOLDS - 阈值配置文件路径 (默认: scripts/regression_thresholds.yaml)
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 默认配置
BASELINE_DIR="${WORKSPACE_DIR}/perf_reports/baselines"
THRESHOLD_FILE="${SCRIPT_DIR}/regression_thresholds.yaml"
EVAL_SCRIPT="${WORKSPACE_DIR}/perf_reports/scripts/evaluate_perception_metrics.py"
RESULTS_DIR="${WORKSPACE_DIR}/perf_reports/results"

# 参数解析
BAG_FILE=""
BASELINE_FILE=""
USE_LATEST_BASELINE=false
GENERATE_BASELINE=false

usage() {
    local exit_code="${1:-0}"
    cat << EOF
Usage: $(basename "$0") <bag_file> [options]

Options:
    -b, --baseline <file>     指定 baseline JSON 文件对比
    -l, --latest-baseline     使用最新的 baseline 文件
    -g, --generate-baseline   生成新的 baseline (不进行对比)
    -t, --threshold <file>    指定阈值配置文件 (默认: ${THRESHOLD_FILE})
    -h, --help                显示帮助

Examples:
    # 生成 baseline
    $(basename "$0") /data/bags/test_track.bag --generate-baseline

    # 与指定 baseline 对比
    $(basename "$0") /data/bags/test_track.bag --baseline /path/to/baseline.json

    # 使用最新 baseline
    $(basename "$0") /data/bags/test_track.bag --latest-baseline
EOF
    exit "${exit_code}"
}

# 解析参数
if [[ $# -eq 0 ]]; then
    usage 1
fi

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--baseline)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --baseline requires a file path"
                usage 1
            fi
            BASELINE_FILE="$2"
            shift 2
            ;;
        -l|--latest-baseline)
            USE_LATEST_BASELINE=true
            shift
            ;;
        -g|--generate-baseline)
            GENERATE_BASELINE=true
            shift
            ;;
        -t|--threshold)
            if [[ $# -lt 2 ]]; then
                echo "ERROR: --threshold requires a file path"
                usage 1
            fi
            THRESHOLD_FILE="$2"
            shift 2
            ;;
        -h|--help)
            usage 0
            ;;
        -* )
            echo "Unknown option: $1"
            usage 1
            ;;
        *)
            if [[ -z "${BAG_FILE}" ]]; then
                BAG_FILE="$1"
                shift
            else
                echo "ERROR: Unexpected positional argument: $1"
                usage 1
            fi
            ;;
    esac
done

if [[ -z "${BAG_FILE}" ]]; then
    echo "ERROR: bag_file is required"
    usage 1
fi

# 检查 bag 文件
if [[ ! -f "${BAG_FILE}" ]]; then
    echo "ERROR: Bag file not found: ${BAG_FILE}"
    exit 1
fi

BAG_NAME=$(basename "${BAG_FILE}" .bag)
mkdir -p "${RESULTS_DIR}"
mkdir -p "${BASELINE_DIR}"

# 查找最新的 baseline
find_latest_baseline() {
    local pattern="${BASELINE_DIR}/${BAG_NAME}_baseline_*.json"
    ls -t ${pattern} 2>/dev/null | head -1
}

# 确定 baseline 文件
if [[ "${GENERATE_BASELINE}" == true ]]; then
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    BASELINE_FILE="${BASELINE_DIR}/${BAG_NAME}_baseline_${TIMESTAMP}.json"
    echo "[INFO] Will generate new baseline: ${BASELINE_FILE}"
elif [[ "${USE_LATEST_BASELINE}" == true ]]; then
    BASELINE_FILE=$(find_latest_baseline)
    if [[ -z "${BASELINE_FILE}" ]]; then
        echo "ERROR: No baseline found for ${BAG_NAME}"
        echo "       Run with --generate-baseline first"
        exit 1
    fi
    echo "[INFO] Using latest baseline: ${BASELINE_FILE}"
elif [[ -n "${BASELINE_FILE}" ]]; then
    if [[ ! -f "${BASELINE_FILE}" ]]; then
        echo "ERROR: Baseline file not found: ${BASELINE_FILE}"
        exit 1
    fi
    echo "[INFO] Using specified baseline: ${BASELINE_FILE}"
else
    # 尝试自动查找 baseline
    BASELINE_FILE=$(find_latest_baseline)
    if [[ -n "${BASELINE_FILE}" ]]; then
        echo "[INFO] Auto-detected baseline: ${BASELINE_FILE}"
    else
        echo "ERROR: No baseline specified and no auto-detected baseline found"
        echo "       Run with --generate-baseline first, or specify --baseline"
        exit 1
    fi
fi

# 检查评估脚本
if [[ ! -f "${EVAL_SCRIPT}" ]]; then
    echo "ERROR: Evaluation script not found: ${EVAL_SCRIPT}"
    exit 1
fi

# 运行评估
RESULT_FILE="${RESULTS_DIR}/${BAG_NAME}_$(date +%Y%m%d_%H%M%S).json"
echo "[INFO] Running perception evaluation..."
echo "       Bag: ${BAG_FILE}"

python3 "${EVAL_SCRIPT}" "${BAG_FILE}" -o "${RESULT_FILE}"

if [[ ! -f "${RESULT_FILE}" ]]; then
    echo "ERROR: Evaluation failed, no result file generated"
    exit 1
fi

echo "[INFO] Result saved to: ${RESULT_FILE}"

# 生成 baseline 模式
if [[ "${GENERATE_BASELINE}" == true ]]; then
    cp "${RESULT_FILE}" "${BASELINE_FILE}"
    echo "[OK] Baseline generated: ${BASELINE_FILE}"
    echo ""
    cat "${BASELINE_FILE}"
    exit 0
fi

# 对比模式
echo "[INFO] Comparing with baseline..."

# 如果没有 Python yaml 模块，使用简单对比
python3 << EOF
import json
import sys
from pathlib import Path

# 加载结果和 baseline
with open("${RESULT_FILE}") as f:
    result = json.load(f)
with open("${BASELINE_FILE}") as f:
    baseline = json.load(f)

# 定义阈值 (可从配置文件加载)
thresholds = {
    "mean_detections": {"abs": 2.0, "rel": 0.15},      # 绝对差值 < 2, 相对 < 15%
    "std_detections": {"abs": 1.0, "rel": 0.20},       # 绝对差值 < 1, 相对 < 20%
    "spike_rate": {"abs": 0.05, "rel": 0.30},          # 绝对差值 < 0.05, 相对 < 30%
    "zero_frame_rate": {"abs": 0.05, "rel": 0.30},     # 绝对差值 < 0.05, 相对 < 30%
    "mean_confidence": {"abs": 0.05, "rel": 0.10},     # 绝对差值 < 0.05, 相对 < 10%
    "mean_distance_m": {"abs": 1.0, "rel": 0.10},      # 绝对差值 < 1m, 相对 < 10%
    "symmetry_ratio": {"abs": 0.1, "rel": 0.15},       # 绝对差值 < 0.1, 相对 < 15%
}

# GT 指标阈值 (如果有)
gt_thresholds = {
    "precision": {"abs": 0.05, "rel": 0.10, "min": 0.85},
    "recall": {"abs": 0.05, "rel": 0.10, "min": 0.80},
    "f1": {"abs": 0.05, "rel": 0.10, "min": 0.82},
    "rmse_m": {"abs": 0.3, "rel": 0.20, "max": 0.5},
}

r_metrics = result.get("metrics", {})
b_metrics = baseline.get("metrics", {})

violations = []
warnings = []

def check_metric(name, current, baseline_val, thresh):
    if current is None or baseline_val is None:
        return
    
    abs_diff = abs(current - baseline_val)
    rel_diff = abs_diff / baseline_val if baseline_val != 0 else float('inf')
    
    status = "OK"
    if abs_diff > thresh.get("abs", float('inf')) and rel_diff > thresh.get("rel", float('inf')):
        status = "FAIL"
        violations.append(f"{name}: {current:.4f} vs {baseline_val:.4f} (abs={abs_diff:.4f}, rel={rel_diff:.2%})")
    elif abs_diff > thresh.get("abs", float('inf')) * 0.5 or rel_diff > thresh.get("rel", float('inf')) * 0.5:
        status = "WARN"
        warnings.append(f"{name}: {current:.4f} vs {baseline_val:.4f} (abs={abs_diff:.4f}, rel={rel_diff:.2%})")
    
    # 检查最小/最大值约束
    if "min" in thresh and current < thresh["min"]:
        violations.append(f"{name}: {current:.4f} < min {thresh['min']}")
    if "max" in thresh and current > thresh["max"]:
        violations.append(f"{name}: {current:.4f} > max {thresh['max']}")
    
    return status

print("\n" + "="*60)
print("REGRESSION TEST RESULTS")
print("="*60)
print(f"Bag:      ${BAG_NAME}")
print(f"Baseline: ${BASELINE_FILE}")
print(f"Result:   ${RESULT_FILE}")
print("-"*60)

# 检查代理指标
print("\nProxy Metrics:")
for name, thresh in thresholds.items():
    current = r_metrics.get(name)
    baseline_val = b_metrics.get(name)
    if current is not None and baseline_val is not None:
        status = check_metric(name, current, baseline_val, thresh)
        marker = "✓" if status == "OK" else ("⚠" if status == "WARN" else "✗")
        print(f"  {marker} {name:25s}: {current:8.4f} vs {baseline_val:8.4f}")

# 检查 GT 指标 (如果有)
has_gt = any(k in r_metrics for k in gt_thresholds.keys())
if has_gt:
    print("\nGT Metrics:")
    for name, thresh in gt_thresholds.items():
        current = r_metrics.get(name)
        baseline_val = b_metrics.get(name)
        if current is not None and baseline_val is not None:
            status = check_metric(name, current, baseline_val, thresh)
            marker = "✓" if status == "OK" else ("⚠" if status == "WARN" else "✗")
            print(f"  {marker} {name:25s}: {current:8.4f} vs {baseline_val:8.4f}")

print("\n" + "-"*60)

if warnings:
    print(f"\n⚠ WARNINGS ({len(warnings)}):")
    for w in warnings:
        print(f"  - {w}")

if violations:
    print(f"\n✗ VIOLATIONS ({len(violations)}):")
    for v in violations:
        print(f"  - {v}")
    print("\n" + "="*60)
    print("RESULT: FAILED")
    print("="*60)
    sys.exit(1)
else:
    print("\n" + "="*60)
    print("RESULT: PASSED")
    print("="*60)
    sys.exit(0)
EOF
