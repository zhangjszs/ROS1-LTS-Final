# 风险清单、监控模板与验收标准 (Risk Checklist)

> **版本**: v1.0
> **状态**: DRAFT
> **前置文档**: `docs/LOCALIZATION_M4_VALIDATION_PLAN.md`

---

## 1. 失效模式监控清单

### 1.1 失效模式总表

| ID | 失效模式 | 严重度 | 检测方法 | 监控频率 |
|----|----------|--------|----------|----------|
| F1 | map_mode 未生效 | 高 | 启动日志检查 | 每次启动 |
| F2 | FG shadow 发散 | 中 | mapper vs FG 位姿差 | 实时 (1 Hz) |
| F3 | INS 漂移无锥桶修正 | 高 | position_rms_m 趋势 | 实时 (1 Hz) |
| F4 | 时间戳未对齐 | 中 | sync_dt_s 分布 | 实时 (10 Hz) |
| F5 | 加速度双重缩放 | 高 | Ax/Ay 幅值检查 | 实时 (100 Hz) |
| F6 | Age 单位不一致 | 中 | GNSS 可用率 | 实时 (1 Hz) |
| F7 | INS 阈值不可配 | 低 | 参数 dump 验证 | 每次启动 |
| F8 | NSV 门控不一致 | 中 | mapper vs FG 质量决策 | 实时 (1 Hz) |
| F9 | 颜色语义缺失 | 中 | color_type 分布统计 | 实时 (10 Hz) |
| F10 | 无重定位能力 | 高 | recovery_time_mean_s | 离线评估 |

### 1.2 各失效模式详细监控

#### F1: map_mode 未生效

**检测**: 启动时检查 ROS 参数

```bash
# 启动后检查
rosparam get /location_node/map/mode
# 预期: "accel" | "skidpad" | "track"
# 异常: 参数不存在或值为空
```

**日志模式**:
```
grep "map_mode" ~/.ros/log/latest/location_node*.log
# 预期: "Map mode set to: <mode>"
```

**修复状态**: 已修复（P0#1），各赛事 YAML 已添加 `mode` 字段。

#### F2: FG shadow 发散

**检测**: 比较 mapper 和 FG 输出位姿

```bash
# 实时监控 (需 shadow mode 日志)
grep "FG shadow" ~/.ros/log/latest/location_node*.log | \
  awk '{print $NF}' | # 提取位姿差
  python3 -c "import sys; vals=[float(l) for l in sys.stdin]; \
    print(f'mean={sum(vals)/len(vals):.3f} max={max(vals):.3f}')"
```

**阈值**:
- 正常: 位姿差 < 1.0m
- 警告: 位姿差 1.0 - 3.0m
- 异常: 位姿差 > 3.0m

#### F3: INS 漂移无锥桶修正

**检测**: 监控 position_rms_m 随时间的趋势

```bash
# 离线评估
python3 perf_reports/scripts/evaluate_localization_metrics.py \
  --csv localization_eval.csv --label drift_check
```

**阈值**:
- 正常: 累积漂移 < 0.5m/圈
- 警告: 累积漂移 0.5 - 1.0m/圈
- 异常: 累积漂移 > 1.0m/圈

#### F4: 时间戳未对齐

**检测**: 监控 INS-LiDAR 同步偏差

```bash
# 从评估 CSV 中检查
python3 -c "
import pandas as pd
df = pd.read_csv('localization_eval.csv')
dt = df['sync_dt_s']
print(f'mean={dt.mean():.4f}s p95={dt.quantile(0.95):.4f}s max={dt.max():.4f}s')
"
```

**阈值**:
- 正常: p95 < 0.02s
- 警告: p95 0.02 - 0.05s
- 异常: p95 > 0.05s

**修复状态**: 已修复（P0#2），INS 环形缓冲区 + 插值已实现。

#### F5: 加速度双重缩放

**检测**: 运行时检查加速度幅值

```bash
rostopic echo /localization/car_state -n 100 | \
  grep -E "Ax|Ay" | awk '{print $2}' | \
  python3 -c "import sys; vals=[abs(float(l)) for l in sys.stdin]; \
    print(f'max_accel={max(vals):.2f} m/s²')"
# 预期: 静止时 < 1.0 m/s²，行驶时 < 15.0 m/s²
# 异常: 静止时 > 5.0 m/s² (说明有缩放问题)
```

**修复状态**: 已修复（P0#3），移除了 ×9.79 双重缩放。

#### F6: Age 单位不一致

**检测**: 监控 GNSS 可用率

```bash
# 检查 GNSS 因子添加频率
grep "GNSS factor" ~/.ros/log/latest/location_node*.log | \
  grep -c "quality=GOOD\|quality=MEDIUM"
# 与总帧数对比，计算可用率
```

**修复状态**: 已修复（P0#4），统一使用 0.1s 单位。

#### F7-F10: 类似模式，见下方监控模板。

---

## 2. 话题/日志观测模板

### 2.1 话题健康检查

```bash
#!/bin/bash
# localization_health_check.sh

echo "=== Localization Topic Health Check ==="

# 检查话题频率
topics=(
  "localization/car_state:100"
  "localization/cone_map:10"
  "localization/global_map:10"
  "localization/pose:100"
  "localization/odom:100"
)

for entry in "${topics[@]}"; do
  topic="${entry%%:*}"
  expected="${entry##*:}"
  hz=$(rostopic hz "/$topic" -w 5 2>&1 | grep "average rate" | awk '{print $3}')
  if [ -z "$hz" ]; then
    echo "[FAIL] $topic: no messages"
  elif (( $(echo "$hz < $expected * 0.8" | bc -l) )); then
    echo "[WARN] $topic: ${hz}Hz (expected ${expected}Hz)"
  else
    echo "[ OK ] $topic: ${hz}Hz"
  fi
done

# 检查 TF
echo ""
echo "=== TF Check ==="
timeout 2 rosrun tf tf_echo world base_link 2>&1 | head -5
```

### 2.2 日志 grep 模板

```bash
#!/bin/bash
# localization_log_check.sh
LOG_DIR="${1:-$HOME/.ros/log/latest}"

echo "=== Localization Log Analysis ==="

# 1. 启动参数确认
echo "--- Startup Parameters ---"
grep -E "map_mode|backend|Map mode" "$LOG_DIR"/location_node*.log | head -5

# 2. 错误和警告
echo "--- Errors & Warnings ---"
grep -cE "ERROR|WARN" "$LOG_DIR"/location_node*.log
grep -E "ERROR" "$LOG_DIR"/location_node*.log | tail -5

# 3. FG shadow 状态
echo "--- FG Shadow Status ---"
grep "FG shadow" "$LOG_DIR"/location_node*.log | tail -3

# 4. 优化耗时
echo "--- Optimization Timing ---"
grep "opt_time" "$LOG_DIR"/location_node*.log | \
  awk '{print $NF}' | \
  python3 -c "
import sys
vals = [float(l) for l in sys.stdin if l.strip()]
if vals:
    print(f'  mean={sum(vals)/len(vals):.2f}ms max={max(vals):.2f}ms')
else:
    print('  No timing data found')
"

# 5. GNSS 质量分布
echo "--- GNSS Quality ---"
for q in GOOD MEDIUM POOR INVALID; do
  count=$(grep -c "gnss_quality=$q" "$LOG_DIR"/location_node*.log 2>/dev/null || echo 0)
  echo "  $q: $count"
done
```

### 2.3 RViz 可视化检查

| 检查项 | Marker/Topic | 预期 |
|--------|-------------|------|
| 车辆位姿 | `localization/pose` | 平滑移动，无跳变 |
| 锥桶地图 | `localization/cone_map` | 锥桶分布合理，无重叠 |
| 全局点云 | `localization/global_map` | 与车辆轨迹一致 |
| TF 树 | `world → base_link` | 连续更新，无断裂 |
| 轨迹 | `/localization/odom` path | 闭合回路无明显漂移 |

---

## 3. 赛事验收标准

### 3.1 Acceleration 模式

| 检查项 | 通过标准 | 检测方法 |
|--------|----------|----------|
| 横向误差 | < 0.3m RMS | `evaluate_localization_metrics.py` |
| 航向误差 | < 2.0° RMS | 同上 |
| 完成率 | 100% (75m 全程) | 轨迹终点检查 |
| 锥桶地图 | 左右各 ≥ 90% 检出 | 地图锥桶数 vs 预期数 |
| 实时性 | 单帧 < 5ms (p99) | `opt_time_ms` 日志 |
| 启动参数 | `map_mode = accel` | `rosparam get` |

### 3.2 Skidpad 模式

| 检查项 | 通过标准 | 检测方法 |
|--------|----------|----------|
| 横向误差 | < 0.5m RMS | `evaluate_localization_metrics.py` |
| 交叉区域误差 | < 0.5m 峰值 | 交叉区域段提取 |
| 圈间漂移 | < 0.3m/圈 | 同一位置多圈对比 |
| 4 圈完成 | 是 | 圈数计数 |
| 圆拟合残差 | < 2.0m | 路标到圆周距离 |
| 启动参数 | `map_mode = skidpad` | `rosparam get` |

### 3.3 TrackDrive 模式

| 检查项 | 通过标准 | 检测方法 |
|--------|----------|----------|
| 横向误差 | < 0.5m RMS | `evaluate_localization_metrics.py` |
| 航向误差 | < 3.0° RMS | 同上 |
| 累积漂移 | < 1.0m/圈 | 圈终点偏差 |
| 恢复时间 | < 2.0s | `recovery_time_mean_s` |
| 地图一致性 | > 90% | 重访区域 IoU |
| 锥桶去重 | 无 < 0.6m 间距重复 | 最近邻距离检查 |
| 启动参数 | `map_mode = track` | `rosparam get` |

### 3.4 通用要求

| 检查项 | 通过标准 | 检测方法 |
|--------|----------|----------|
| 话题频率 | car_state ≥ 80Hz, cone_map ≥ 8Hz | `rostopic hz` |
| TF 发布 | world→base_link 连续 | `tf_echo` |
| 无崩溃 | 全程无 node 退出 | `rosnode list` |
| 内存 | < 200MB RSS | `ps aux` |
| CPU | < 50% 单核 | `top` |

---

## 4. Go/No-Go 决策矩阵

### 4.1 竞赛部署检查表

| 序号 | 检查项 | GO 条件 | NO-GO 条件 | 负责人 |
|------|--------|---------|-----------|--------|
| 1 | 所有赛事验收标准 | 全部通过 | 任一失败 | 定位组 |
| 2 | 回归门控 | 全部通过 | 任一失败 | CI/CD |
| 3 | Shadow mode 对比 | FG ≤ mapper | FG > mapper × 1.2 | 定位组 |
| 4 | 实时性 (ARM) | p99 < 5ms | p99 > 8ms | 定位组 |
| 5 | 异常恢复 | recovery < 2s | recovery > 5s | 定位组 |
| 6 | 验证 bag 数量 | ≥ 3 bags/赛事 | < 3 bags/赛事 | 测试组 |
| 7 | 启动脚本验证 | 参数正确加载 | 参数缺失/错误 | 集成组 |
| 8 | 硬件在环测试 | 通过 | 未执行 | 车辆组 |

### 4.2 决策流程

```
1. 定位组完成所有赛事模式验收 → 提交验收报告
2. CI/CD 自动运行回归门控 → 生成 regression_gate_result
3. 集成组执行硬件在环测试 → 确认实车表现
4. 团队评审 (≥ 2 人) → 签署 Go/No-Go
5. 如果 GO: 部署到竞赛车辆
   如果 NO-GO: 记录失败项，制定修复计划
```

---

## 5. 监控仪表盘模板

### 5.1 实时监控指标

```
┌─────────────────────────────────────────────────┐
│           Localization Health Dashboard          │
├─────────────────────────────────────────────────┤
│ State: [TRACKING]  Backend: [mapper]            │
│ GNSS:  [GOOD]      Satellites: [14]             │
├─────────────────────────────────────────────────┤
│ Position:                                       │
│   x: 123.45m  y: 67.89m  θ: 1.23rad           │
│   V: 8.5m/s   Wz: 0.12rad/s                   │
├─────────────────────────────────────────────────┤
│ Factor Graph (shadow):                          │
│   Keyframes: 42    Landmarks: 85               │
│   chi²: 15.3      opt_time: 2.1ms             │
│   FG-Mapper Δ: 0.23m                           │
├─────────────────────────────────────────────────┤
│ Cone Map:                                       │
│   Total: 85  Local: 23  Match ratio: 0.87      │
│   Blue: 35  Yellow: 38  Orange: 8  None: 4     │
├─────────────────────────────────────────────────┤
│ Topics:                                         │
│   car_state: 98Hz ✓  cone_map: 10Hz ✓          │
│   pose: 98Hz ✓       odom: 98Hz ✓              │
│   TF: 98Hz ✓                                    │
└─────────────────────────────────────────────────┘
```

### 5.2 离线评估报告模板

```markdown
# Localization Evaluation Report

## Test Info
- Date: YYYY-MM-DD
- Bag: <bag_path>
- Mission: <accel|skidpad|track>
- Backend: <mapper|factor_graph>
- Duration: XX.Xs

## KPI Summary
| Metric | Value | Threshold | Status |
|--------|-------|-----------|--------|
| position_rms_m | X.XXm | <0.5m | PASS/FAIL |
| heading_rms_deg | X.X° | <3.0° | PASS/FAIL |
| cumulative_drift_m | X.Xm | <1.0m/lap | PASS/FAIL |
| recovery_time_mean_s | X.Xs | <2.0s | PASS/FAIL |
| opt_time_ms (p99) | X.Xms | <5.0ms | PASS/FAIL |

## Regression Gate
- Baseline: <baseline_label>
- Result: PASS/FAIL
- Details: <regression_gate_result.md>

## Notes
- <observations, anomalies, recommendations>
```

---

## 6. 定期维护检查表

### 6.1 每次测试前

- [ ] 确认 `map_mode` 与赛事匹配
- [ ] 确认 `backend` 参数正确
- [ ] 确认 INS 设备已校准
- [ ] 确认 LiDAR 正常出点
- [ ] 确认 rosbag 录制已启动

### 6.2 每次测试后

- [ ] 运行 `build_localization_eval_csv_from_bag.py`
- [ ] 运行 `evaluate_localization_metrics.py`
- [ ] 检查验收标准是否通过
- [ ] 如有异常，运行 `localization_log_check.sh`
- [ ] 归档 bag 文件（按命名规范）

### 6.3 每周

- [ ] 运行回归门控对比最新 baseline
- [ ] 检查 shadow mode FG-mapper 偏差趋势
- [ ] 更新 baseline（如有改进）
- [ ] 清理过期 bag 文件
