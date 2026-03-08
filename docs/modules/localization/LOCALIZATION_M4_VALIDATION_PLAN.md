# 验证计划、场景矩阵与数据集协议 (Validation Plan)

> **版本**: v1.0
> **状态**: DRAFT
> **前置文档**: `docs/LOCALIZATION_M2_CTF_GRAPH_DESIGN.md`
> **工具基线**: `perf_reports/scripts/evaluate_localization_metrics.py`

---

## 1. 验证目标

为定位系统的 shadow mode → 主后端迁移提供系统化的验证框架：

- **场景覆盖**: 8 类典型场景 × 3 种赛事模式
- **KPI 量化**: 6 项核心指标，每项有明确的通过/失败阈值
- **数据集规范**: rosbag 录制、命名、标注的标准化流程
- **回归门控**: 自动化 baseline 对比，防止性能退化

---

## 2. 场景矩阵

### 2.1 场景定义

| ID | 场景 | 描述 | 影响的子系统 |
|----|------|------|-------------|
| S1 | 稀疏锥桶 | 锥桶间距 > 5m，部分锥桶缺失 | 数据关联、地图构建 |
| S2 | 颜色误检 | LiDAR 颜色分类错误或返回 NONE | 颜色因子、拓扑约束 |
| S3 | 首圈建图 | 无先验地图，纯增量建图 | 路标初始化、FG 收敛 |
| S4 | Skidpad 交叉区 | 8 字形交叉点的关联歧义 | 回环检测、方向感知 |
| S5 | 高速弯道 | 急弯 + 蛇形，时间同步压力大 | INS 预积分、时间戳对齐 |
| S6 | GNSS 退化 | 卫星数下降或差分龄期增大 | GNSS 因子权重、降级行为 |
| S7 | 堆叠锥桶 | 备用锥桶堆叠产生幽灵路标 | 去重逻辑、路标管理 |
| S8 | 系统重启 | 运行中重启，需冷启动重定位 | 重定位状态机、粒子重采样 |

### 2.2 场景 × 赛事 × 指标矩阵

```
           position  heading  cumulative  recovery  map_consist  assoc
           _rms_m    _rms_deg _drift_m    _time_s   ency         _accuracy
S1-accel   ●         ●        ○           ○         ●            ●
S1-skid    ●         ●        ○           ○         ●            ●
S1-track   ●         ●        ●           ○         ●            ●
S2-accel   ○         ○        ○           ○         ●            ●
S2-skid    ○         ○        ○           ○         ●            ●
S2-track   ○         ○        ○           ○         ●            ●
S3-accel   ●         ●        ○           ○         ●            ○
S3-skid    ●         ●        ○           ○         ●            ○
S3-track   ●         ●        ●           ○         ●            ○
S4-skid    ●         ●        ●           ○         ●            ●
S5-track   ●         ●        ●           ○         ○            ○
S6-all     ●         ●        ●           ●         ○            ○
S7-all     ○         ○        ○           ○         ●            ●
S8-all     ○         ○        ○           ●         ○            ○

● = 主要关注指标    ○ = 次要/不适用
```

---

## 3. KPI 定义

### 3.1 核心指标

| 指标 | 定义 | 计算方法 |
|------|------|----------|
| `position_rms_m` | 位置 RMS 误差 | `sqrt(mean((est_x - ref_x)² + (est_y - ref_y)²))` |
| `heading_rms_deg` | 航向 RMS 误差 | `sqrt(mean(angle_wrap(est_yaw - ref_yaw)²))` × 180/π |
| `cumulative_drift_m` | 累积漂移 | 每圈终点与起点的位置偏差 |
| `recovery_time_mean_s` | 平均恢复时间 | 从 LOST 到 TRACKING 的平均耗时 |
| `map_consistency` | 地图一致性 | 重访区域路标重叠率 (IoU) |
| `association_accuracy` | 关联准确率 | 正确锥桶-路标匹配数 / 总匹配数 |

### 3.2 辅助指标

| 指标 | 定义 | 用途 |
|------|------|------|
| `chi2_normalized` | 归一化因子残差 | 异常状态机调参 |
| `cone_match_ratio` | 锥桶匹配率 | 降级触发调参 |
| `opt_time_ms` | 单次优化耗时 | 实时性验证 |
| `landmark_count` | 路标总数 | 内存/计算预算 |
| `gnss_availability` | GNSS 可用率 | GNSS 因子贡献度 |
| `sync_dt_s` | INS-LiDAR 同步偏差 | 时间戳对齐质量 |

---

## 4. 验收标准

### 4.1 Acceleration 模式

| 指标 | 通过 | 警告 | 失败 |
|------|------|------|------|
| `position_rms_m` | < 0.3 | 0.3 - 0.5 | > 0.5 |
| `heading_rms_deg` | < 2.0 | 2.0 - 5.0 | > 5.0 |
| 完成率 | 100% | — | < 100% |
| `opt_time_ms` | < 5.0 | 5.0 - 8.0 | > 8.0 |

### 4.2 Skidpad 模式

| 指标 | 通过 | 警告 | 失败 |
|------|------|------|------|
| `position_rms_m` | < 0.5 | 0.5 - 0.8 | > 0.8 |
| `heading_rms_deg` | < 3.0 | 3.0 - 5.0 | > 5.0 |
| 交叉区域横向误差 | < 0.5 | 0.5 - 1.0 | > 1.0 |
| 圈间漂移 | < 0.3 | 0.3 - 0.5 | > 0.5 |
| 4 圈完成 | 是 | — | 否 |

### 4.3 TrackDrive / Autocross 模式

| 指标 | 通过 | 警告 | 失败 |
|------|------|------|------|
| `position_rms_m` | < 0.5 | 0.5 - 1.0 | > 1.0 |
| `heading_rms_deg` | < 3.0 | 3.0 - 5.0 | > 5.0 |
| `cumulative_drift_m` | < 1.0/圈 | 1.0 - 2.0/圈 | > 2.0/圈 |
| `recovery_time_mean_s` | < 2.0 | 2.0 - 5.0 | > 5.0 |
| `map_consistency` | > 90% | 80% - 90% | < 80% |

### 4.4 通用实时性要求

| 指标 | 通过 | 失败 |
|------|------|------|
| 预积分累加 (100 Hz) | < 0.01 ms | > 0.1 ms |
| 数据关联 (10 Hz) | < 0.5 ms | > 2.0 ms |
| iSAM2 更新 (~2 Hz) | < 3.0 ms | > 5.0 ms |
| 路标剪枝 (~2 Hz) | < 0.5 ms | > 2.0 ms |
| 单次关键帧总计 | < 5.0 ms | > 8.0 ms |

---

## 5. 数据集协议

### 5.1 Rosbag 录制要求

#### 必录话题

| 话题 | 消息类型 | 频率 | 用途 |
|------|----------|------|------|
| `/pbox_pub/Ins` | `HUAT_InsP2` | 100 Hz | INS 输入 + RTK 参考轨迹 |
| `/velodyne_points` | `PointCloud2` | 10 Hz | 原始点云（离线重处理） |
| `perception/lidar_cluster/detections` | `HUAT_ConeDetections` | 10 Hz | 锥桶检测输入 |
| `/localization/car_state` | `HUAT_CarState` | 100 Hz | 定位输出（评估对象） |
| `/localization/cone_map` | `HUAT_ConeMap` | 10 Hz | 锥桶地图（一致性评估） |
| `/tf` | `tf2_msgs/TFMessage` | ~100 Hz | TF 连续性与外推风险 |
| `/localization/diagnostics` | `DiagnosticArray` | 1-10 Hz | 匹配率/状态机/漂移恢复统计 |

#### 可选话题

| 话题 | 用途 |
|------|------|
| `/localization/car_state_fg` | FG 副输出（Phase 2 双输出模式） |
| `/localization/odom` | 里程计（漂移分析） |
| `/tf` | 坐标变换（可视化回放） |

### 5.2 命名规范

```
{date}_{mission}_{scenario}_{vehicle}_{seq}.bag

示例:
20260210_track_normal_A13_001.bag
20260210_skidpad_crossover_A13_002.bag
20260210_accel_sparse_A13_001.bag
20260210_track_gnss_degrade_A13_003.bag
```

### 5.3 地面真值

**主要来源**: RTK-GNSS 后处理轨迹

- INS 设备在 Status=2 时提供 RTK 级精度（~2cm）
- 使用 `HUAT_InsP2` 中的 `Lat/Lon/Heading` 作为参考
- 通过 `build_localization_eval_csv_from_bag.py` 提取并对齐

**锥桶地面真值**（可选，用于 `association_accuracy`）:

- 赛前用全站仪或 RTK 测量锥桶位置
- 存储为 CSV: `cone_id, x_enu, y_enu, color_type`
- 坐标系与定位输出一致（首次 INS 有效位置为原点）

### 5.4 场景注入方法

对于无法自然采集的场景，通过离线注入模拟：

| 场景 | 注入方法 |
|------|----------|
| S1 稀疏锥桶 | 随机丢弃 30%/50% 的锥桶检测 |
| S2 颜色误检 | 随机翻转 10%/20%/50% 的 `color_types` |
| S6 GNSS 退化 | 将 INS `Status` 设为 1，`NSV1/NSV2` 设为 0 |
| S7 堆叠锥桶 | 在已有锥桶位置附近 (±0.3m) 注入重复检测 |
| S8 系统重启 | 截断 bag 前 N 秒，模拟冷启动 |

注入脚本: `perf_reports/scripts/inject_scenario.py`（待开发）

---

## 6. 评估流程

### 6.1 端到端流程

```
1. 录制 rosbag (按 §5.1 要求)
     ↓
2. 提取对齐 CSV
   $ python3 perf_reports/scripts/build_localization_eval_csv_from_bag.py \
       --bag <bag_path> \
       --est-topic /localization/car_state \
       --ref-topic /pbox_pub/Ins \
       --output docs/baseline/localization_eval.csv
     ↓
3. 计算指标
   $ python3 perf_reports/scripts/evaluate_localization_metrics.py \
       --csv docs/baseline/localization_eval.csv \
       --label "baseline_v1" \
       --output-json docs/baseline/localization_baseline.json \
       --output-md docs/baseline/localization_metrics.md
     ↓
4. 回归门控 (与 baseline 对比)
   $ python3 perf_reports/scripts/check_regression_gate.py \
       --baseline docs/baseline/localization_baseline.json \
       --candidate docs/candidate/localization_candidate.json \
       --gate-config docs/baseline/rollback_gate.yaml
     ↓
5. 生成报告
   $ python3 perf_reports/scripts/generate_baseline_dashboard.py
```

### 6.2 无 GT 代理指标快速评估（新增）

用于仅依赖 bag 的代理闭环指标（无真值）：

```bash
python3 perf_reports/scripts/evaluate_localization_bag_proxy_metrics.py \
  --bag <bag_path> \
  --car-state-topic localization/car_state \
  --cone-map-topic localization/cone_map \
  --detections-topic perception/lidar_cluster/detections \
  --tf-topic /tf \
  --ins-topic sensors/ins \
  --diagnostics-topic localization/diagnostics \
  --output-json docs/baseline/localization_proxy_metrics.json \
  --output-md docs/baseline/localization_proxy_metrics.md
```

输出最小指标：`match_success_rate`、`non_tracking_ratio`、
`relocalization_success_rate/latency`、`position_jitter_m`、
`closure_error_m`、`map_repeat_consistency_m`、`mean_match_distance_std_m`。

### 6.3 FG vs Mapper A/B 对比（Phase 2）

在双输出模式下，同时评估两个后端：

```
# 提取 mapper 输出
$ python3 build_localization_eval_csv_from_bag.py \
    --est-topic /localization/car_state \
    --output mapper_eval.csv

# 提取 FG 输出
$ python3 build_localization_eval_csv_from_bag.py \
    --est-topic /localization/car_state_fg \
    --output fg_eval.csv

# 分别计算指标
$ python3 evaluate_localization_metrics.py --csv mapper_eval.csv --label mapper
$ python3 evaluate_localization_metrics.py --csv fg_eval.csv --label factor_graph

# 对比
$ python3 check_regression_gate.py \
    --baseline mapper_baseline.json \
    --candidate fg_baseline.json
```

### 6.4 回归门控配置

`docs/baseline/rollback_gate.yaml`:

```yaml
gates:
  position_rms_m:
    regression_pct: 10    # 允许最多 10% 退化
    absolute_max: 1.0     # 绝对上限 1.0m
  heading_rms_deg:
    regression_pct: 10
    absolute_max: 5.0
  recovery_time_mean_s:
    regression_pct: 20
    absolute_max: 5.0
  opt_time_ms:
    regression_pct: 15
    absolute_max: 8.0
```

---

## 7. Go/No-Go 决策矩阵

### 7.1 竞赛部署决策

| 条件 | GO | NO-GO |
|------|-----|-------|
| 所有赛事模式验收标准 | 全部通过 | 任一失败 |
| 回归门控 | 全部通过 | 任一失败 |
| Shadow mode 对比 | FG ≤ mapper 误差 | FG > mapper 误差 × 1.2 |
| 实时性 | 单帧 < 5ms (p99) | 单帧 > 8ms (p99) |
| 异常恢复 | recovery_time < 2s | recovery_time > 5s |
| 最少验证 bag 数 | ≥ 3 bags/赛事 | < 3 bags/赛事 |

### 7.2 后端切换决策

从 mapper 切换到 FG 主后端的前提条件：

1. **Phase 1 完成**: Shadow mode 运行 ≥ 10 个 bag，无崩溃
2. **Phase 2 完成**: 双输出模式下 FG 指标 ≤ mapper 指标（所有赛事）
3. **异常状态机验证**: LOST → TRACKING 恢复成功率 > 90%
4. **实时性验证**: ARM Cortex-A72 上 p99 < 5ms
5. **团队评审**: 至少 2 人 code review 通过

---

## 8. 待开发工具

| 工具 | 描述 | 优先级 |
|------|------|--------|
| `inject_scenario.py` | 场景注入脚本（锥桶丢弃、颜色翻转、GNSS 退化） | P1 |
| `compare_backends.py` | mapper vs FG 自动对比报告 | P1 |
| `cone_gt_annotator.py` | 锥桶地面真值标注工具 | P2 |
| `localization_regression_ci.yaml` | GitHub Actions CI 集成 | P2 |
