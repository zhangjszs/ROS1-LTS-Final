#!/bin/bash
# localization_health_check.sh - 定位模块健康检查脚本
# 检查定位系统的运行状态、延迟、重定位状态等关键指标

set -euo pipefail

echo "=== 定位模块健康检查 ==="
echo "检查时间: $(date)"
echo ""

# 检查ROS核心是否运行
if ! rostopic list > /dev/null 2>&1; then
    echo "❌ 错误: ROS核心未运行"
    exit 1
fi
echo "✅ ROS核心运行正常"

# 检查定位相关话题是否存在
REQUIRED_TOPICS=(
    "/localization/car_state"
    "/localization/diagnostics"
    "/perception/lidar_cluster/detections"
    "/INS/ASENSING_INS"
)

echo ""
echo "--- 话题状态检查 ---"
all_topics_ok=true
for topic in "${REQUIRED_TOPICS[@]}"; do
    if rostopic list | grep -q "^$topic$"; then
        # 检查话题发布频率
        hz=$(rostopic hz -w 3 $topic 2>&1 | head -n 5 | grep "average rate:" | awk '{print $3}')
        if [ -z "$hz" ]; then
            echo "⚠️  $topic 存在但没有数据"
            all_topics_ok=false
        else
            echo "✅ $topic 正常, 频率: ${hz}Hz"
        fi
    else
        echo "❌ $topic 不存在"
        all_topics_ok=false
    fi
done

# 检查定位状态
echo ""
echo "--- 定位状态检查 ---"
if rostopic echo /localization/diagnostics -n 1 2>/dev/null | grep -q "TRACKING"; then
    echo "✅ 定位状态: TRACKING (正常跟踪)"
elif rostopic echo /localization/diagnostics -n 1 2>/dev/null | grep -q "DEGRADED"; then
    echo "⚠️  定位状态: DEGRADED (精度下降)"
elif rostopic echo /localization/diagnostics -n 1 2>/dev/null | grep -q "INS_ONLY"; then
    echo "❌ 定位状态: INS_ONLY (仅惯导工作，无激光匹配)"
else
    echo "ℹ️  无法获取定位状态，可能模块未启动"
fi

# 检查端到端延迟
echo ""
echo "--- 性能指标检查 ---"
e2e_latency=$(rostopic echo /localization/diagnostics -n 1 2>/dev/null | grep "e2e_latency_mean_ms" | awk '{print $2}')
if [ -n "$e2e_latency" ]; then
    echo "✅ 端到端平均延迟: ${e2e_latency}ms"
    if (( $(echo "$e2e_latency > 100" | bc -l) )); then
        echo "⚠️  延迟过高，超过100ms阈值"
    fi
fi

reloc_success=$(rostopic echo /localization/diagnostics -n 1 2>/dev/null | grep "reloc_success_count" | awk '{print $2}')
reloc_attempt=$(rostopic echo /localization/diagnostics -n 1 2>/dev/null | grep "reloc_attempt_count" | awk '{print $2}')
if [ -n "$reloc_attempt" ] && [ "$reloc_attempt" -gt 0 ]; then
    rate=$(echo "scale=2; $reloc_success / $reloc_attempt * 100" | bc)
    echo "✅ 重定位成功率: ${rate}% ($reloc_success/$reloc_attempt)"
fi

echo ""
echo "--- 总结 ---"
if $all_topics_ok; then
    echo "✅ 所有必要话题均正常"
else
    echo "❌ 部分话题存在问题，请检查"
fi
echo ""
echo "检查完成！"