# CTF-Graph 算法设计规格 (Cone-Topology Factor Graph)

> **版本**: v1.0
> **状态**: DRAFT
> **前置文档**: `docs/LOCALIZATION_M1_INTERFACE_FREEZE.md`
> **代码基线**: `localization_core/factor_graph_optimizer.hpp` (shadow mode)

---

## 1. 设计目标

将现有 shadow-mode 因子图后端升级为主定位后端，实现：

- **锥桶拓扑约束**: 利用赛道左蓝右黄的颜色拓扑关系作为软约束
- **赛事几何先验**: 针对 accel/skidpad/track 三种赛事注入结构化先验
- **鲁棒数据关联**: Mahalanobis + 颜色一致性 + 拓扑一致性的多维代价
- **异常检测与恢复**: Normal → Degraded → Lost → Relocalization 状态机
- **CPU 实时性**: 单次 iSAM2 更新 < 5ms @ ARM Cortex-A72

---

## 2. 状态向量

### 2.1 分阶段扩展

| 阶段 | 状态向量 | 维度 | 说明 |
|------|----------|------|------|
| **Phase 1** (当前) | `[x, y, θ]` + `[vx, vy]` | 3+2 | Pose2 + Vector2，已实现 |
| **Phase 2** (目标) | `[x, y, θ, vx, vy, b_gz]` | 6 | 增加陀螺仪 z 偏置 |
| **Phase 3** (可选) | `[x, y, θ, vx, vy, b_gz, b_ax, b_ay]` | 8 | 增加加速度计偏置 |

**Phase 2 优先理由**: 陀螺仪偏置 `b_gz` 是航向漂移的主因，加速度计偏置在
当前驱动未发布 acc 字段的情况下无实际意义，待驱动修复后再启用 Phase 3。

### 2.2 GTSAM 符号约定

| 符号 | 前缀 | 含义 | 类型 |
|------|------|------|------|
| `x_k` | `'x'` | 第 k 个关键帧位姿 | `Pose2` |
| `v_k` | `'v'` | 第 k 个关键帧速度 | `Vector2` |
| `b_k` | `'b'` | 第 k 个关键帧偏置 | `Vector1` (Phase 2) |
| `l_j` | `'l'` | 第 j 个路标 (锥桶) | `Point2` |

---

## 3. 因子定义

### 3.1 IMU 运动因子 (BetweenFactor)
### 3.1 IMU 运动因子 (BetweenFactor)

**现有实现**: `addImuFactor()` — 速度+偏航角速率死算积分为 `BetweenFactor<Pose2>`。

**升级方案 (Phase 2)**:

```
预积分模型:
  Δx = Σ v_fwd · cos(Σ(ω_z - b_gz) · dt) · dt
  Δy = Σ v_fwd · sin(Σ(ω_z - b_gz) · dt) · dt
  Δθ = Σ (ω_z - b_gz) · dt

噪声模型:
  σ_xy = sigma_imu_xy × √(Σdt)
  σ_θ  = sigma_imu_theta × √(Σdt)
```

偏置 `b_gz` 通过 `BetweenFactor<Vector1>` 施加随机游走约束：
```
b_gz(k) - b_gz(k-1) ~ N(0, σ_bias_walk² × dt)
σ_bias_walk = 0.001 rad/s/√s  (典型 MEMS 陀螺仪)
```

**代码映射**: 扩展 `addImuFactor()`，在预积分中减去 `b_gz`，
新增 `addBiasFactor()` 方法。

### 3.2 GNSS 弱先验因子 (PriorFactor)

**现有实现**: `addGnssFactor()` — 按 GOOD/MEDIUM/POOR 分级设置 σ，
Cauchy 鲁棒核抑制异常值。

**保持不变**，参数表：

| 质量等级 | 条件 | σ_xy (m) | 鲁棒核 |
|----------|------|----------|--------|
| GOOD | max(NSV1,NSV2) ≥ 12, Age < 5 | 0.5 | Cauchy(5.0) |
| MEDIUM | max(NSV1,NSV2) ≥ 8, Age < 15 | 2.0 | Cauchy(5.0) |
| POOR | 其他 Status ≥ 2 | 10.0 | Cauchy(5.0) |
| INVALID | Status < 2 | — | 不添加因子 |

航向分量 σ_θ = 100 rad（不约束）。

### 3.3 锥桶 Bearing-Range 路标因子

**现有实现**: `addConeFactors()` — `BearingRangeFactor<Pose2, Point2>`，
距离相关噪声，颜色不匹配时增加 range σ。

**升级方案**: 引入颜色软权重矩阵（见 §4.2）替代硬惩罚。

噪声模型：
```
σ_range   = σ_range_base + σ_range_scale × range + color_penalty
σ_bearing = σ_bearing_base + σ_bearing_scale × range
鲁棒核: Huber(2.0)
```

### 3.4 颜色-拓扑软关联因子 (新增, Innovation #1)

**动机**: FSG 规则要求赛道左蓝右黄，但 LiDAR 颜色分类不可靠（尤其远距离），
不应硬门控，而应作为软约束。

**颜色混淆矩阵** `C[obs][map]`:

```
           map: BLUE  YELLOW  ORANGE  NONE
obs: BLUE    0.90    0.05    0.03   0.02
     YELLOW  0.05    0.90    0.03   0.02
     ORANGE  0.10    0.10    0.75   0.05
     NONE    0.25    0.25    0.25   0.25
```

**因子形式**: 对每个锥桶观测-路标关联 (i, j)，附加一元因子：

```
color_cost(i, j) = -log(C[obs_color_i][map_color_j])
```

转化为等效 σ 增量叠加到 range 噪声上：
```
σ_color = sqrt(2 × color_cost) × color_weight
σ_range_total = σ_range + σ_color
```

**拓扑一致性**: 对已关联的相邻锥桶对 (j, j+1)，检查：
- 同侧锥桶颜色应一致（蓝-蓝 或 黄-黄）
- 对侧锥桶颜色应互补（蓝-黄）
- 违反拓扑时额外增加 `σ_topo_penalty`

**代码映射**: 新增 `addColorTopologyFactor()` 方法，在 `addConeFactors()` 中调用。

### 3.5 赛事几何先验因子 (新增)

根据 `map_mode` 注入不同的结构化先验：

#### 3.5.1 Acceleration 模式
- **平行约束**: 锥桶 Y 坐标应在 ±`cone_y_max` 内
- **等间距先验**: 相邻同侧锥桶 X 间距 ≈ `expected_cone_spacing`
- 实现为路标一元因子 `PriorFactor<Point2>` 约束 Y 分量

#### 3.5.2 Skidpad 模式 (Innovation #2)
- **双圆结构因子**: 路标到最近圆心的距离应 ≈ `circle_radius`
- 两个圆心位置: `(0, ±circle_center_dist/2)` (base_link 初始系)
- 实现为自定义一元因子:
```
residual(l_j) = min(|dist(l_j, c1) - R|, |dist(l_j, c2) - R|)
σ_circle = circle_tolerance
```

#### 3.5.3 Track 模式
- **赛道宽度下界**: 对侧锥桶对间距 ≥ `track_width × 0.8`
- 实现为二元因子 `BetweenFactor<Point2>` 的不等式松弛

**代码映射**: 新增 `addGeometryPriorFactors()` 方法，按 `map_mode` 分支。

### 3.6 速度/偏航角速率因子

**现有实现**: `addSpeedFactor()` — `PriorFactor<Vector2>` + Huber 鲁棒核。

**保持不变**。

---

## 4. 数据关联

### 4.1 多维代价函数

对每个观测 `o_i` 和候选路标 `l_j`，计算关联代价：

```
cost(i, j) = w_maha × d_maha(i, j)²
           + w_color × color_cost(i, j)
           + w_topo × topo_cost(i, j)
```

| 权重 | 默认值 | 说明 |
|------|--------|------|
| `w_maha` | 1.0 | Mahalanobis 距离权重 |
| `w_color` | 0.5 | 颜色一致性权重 |
| `w_topo` | 0.3 | 拓扑一致性权重 |

#### Mahalanobis 距离
```
d_maha² = Δp^T × Σ^{-1} × Δp
Δp = [gx_obs - lx_map, gy_obs - ly_map]
Σ = diag(σ_range², σ_bearing² × range²)  旋转到全局系
```

#### 颜色代价
```
color_cost = -log(C[obs_color][map_color])
```

#### 拓扑代价
```
topo_cost = 0                    如果颜色关系符合左蓝右黄
          = topo_penalty (2.0)   如果违反拓扑
```
### 4.2 关联决策

```
if min_cost < gate_threshold:
    associate(o_i, l_j*)        # j* = argmin cost
elif o_i.range < new_landmark_range_max:
    create_new_landmark(o_i)
else:
    discard(o_i)
```

`gate_threshold` 默认 = `merge_distance²`（与现有 KD-tree 合并距离一致）。

### 4.3 堆叠锥桶抑制

**问题**: 同一物理锥桶被多帧重复建图，产生间距极小的"幽灵"路标。

**抑制策略**: 新建路标前检查最近已有路标距离：
```
if nearest_landmark_dist < dedup_radius:
    merge_into_nearest()
else:
    create_new()

dedup_radius = max(0.6, merge_distance × 0.4)
```

与 mapper 中的近邻去重逻辑一致（`location_mapper.cpp:518`）。

**代码映射**: 在 `findOrCreateLandmark()` 中已有 `merge_distance` 检查，
增加 `dedup_radius` 二次检查。

---

## 5. 异常状态机

### 5.1 状态定义

```
┌──────────┐    chi²↑ or GNSS lost    ┌───────────┐
│  NORMAL  │ ─────────────────────────►│ DEGRADED  │
│          │◄───────────────────────── │           │
└──────────┘    chi²↓ and GNSS back   └───────────┘
                                            │
                                            │ cone_match_ratio < 0.2
                                            │ for > 3s
                                            ▼
                                      ┌───────────┐
                                      │   LOST    │
                                      │           │
                                      └─────┬─────┘
                                            │ trigger relocalization
                                            ▼
                                      ┌───────────┐
                                      │  RELOC    │──► NORMAL (if converged)
                                      │           │──► LOST   (if timeout)
                                      └───────────┘
```

### 5.2 状态转移条件

| 转移 | 条件 | 阈值 |
|------|------|------|
| NORMAL → DEGRADED | 最近 5 帧平均 chi² > `chi2_degrade_threshold` | 50.0 |
| NORMAL → DEGRADED | GNSS 质量降为 POOR 或 INVALID | — |
| DEGRADED → NORMAL | 平均 chi² < `chi2_recover_threshold` 且 GNSS ≥ MEDIUM | 20.0 |
| DEGRADED → LOST | `cone_match_ratio` < 0.2 持续 > 3.0s | — |
| DEGRADED → LOST | 连续 10 帧无有效锥桶关联 | — |
| LOST → RELOC | 自动触发重定位 | — |
| RELOC → NORMAL | 重定位收敛（Innovation #3 详细设计） | — |
| RELOC → LOST | 重定位超时 > 5.0s | — |

### 5.3 指标计算

```cpp
// chi² 归一化: 总因子残差 / 因子数
double chi2_normalized = isam_->getFactorsUnsafe().error(result)
                       / isam_->getFactorsUnsafe().size();

// 锥桶匹配率: 成功关联数 / 观测总数
double cone_match_ratio = matched_count / total_obs_count;
```

### 5.4 降级行为

| 状态 | 行为 |
|------|------|
| NORMAL | 正常输出 FG 优化位姿 |
| DEGRADED | 输出 FG 位姿但增大协方差，下游可据此降速 |
| LOST | 回退到 mapper 后端输出，标记 `quality=LOST` |
| RELOC | 暂停 FG 输出，使用 mapper 过渡 |

**代码映射**: 新增 `AnomalyStateMachine` 类，在 `TryUpdate()` 返回后评估。

---

## 6. iSAM2 增量优化策略

### 6.1 关键帧准则

与现有实现一致（`shouldCreateKeyframe()`）：

| 条件 | 阈值 | 配置键 |
|------|------|--------|
| 位移 | ≥ 1.0 m | `fg/keyframe_dist` |
| 偏航 | ≥ 0.1 rad (≈5.7°) | `fg/keyframe_yaw` |
| 时间 | ≥ 0.5 s | `fg/keyframe_dt` |

满足任一条件即创建关键帧。

### 6.2 路标管理

| 操作 | 条件 | 说明 |
|------|------|------|
| **合并** | 两路标距离 < `merge_distance` | 保留观测次数多的，合并位置为加权均值 |
| **剪枝** | 路标数 > `max_landmarks` | 移除观测次数最少的路标 |
| **置信度** | `obs_count` 递增 | 观测次数 > `min_obs_to_keep` 的路标视为稳定 |
| **颜色更新** | 仅 NONE → 有色 | 不允许有色 → 有色覆盖（避免颜色翻转） |

### 6.3 计算预算

| 操作 | 目标耗时 | 约束 |
|------|----------|------|
| 预积分累加 | < 0.01 ms | 每 INS 帧 (100 Hz) |
| 数据关联 | < 0.5 ms | 每锥桶帧 (10 Hz) |
| iSAM2 update | < 3.0 ms | 每关键帧 (~2 Hz) |
| 路标剪枝 | < 0.5 ms | 每关键帧 |
| **总计** | < 5.0 ms | 单次关键帧周期 |

**监控**: `LastOptTimeMs()` 已有计时，超过 5ms 时 `ROS_WARN`。

### 6.4 iSAM2 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `relinearizeThreshold` | 0.1 | 重线性化阈值 |
| `relinearizeSkip` | 1 | 每次更新都检查重线性化 |
| `optimizationParams` | Dogleg | 比 GN 更鲁棒 |
| `run_extra_update` | false | 生产环境关闭，调试时可开启 |

---

## 7. 从 Shadow Mode 到主后端的迁移路径

### 7.1 Phase 1: Shadow Mode 验证 (当前)

```
INS → mapper (主输出)
    → FG (shadow, 仅日志)
```

- 对比 mapper 和 FG 的位姿差异
- 收集 chi²、匹配率、优化耗时统计
- 验证 FG 路标地图与 mapper 点云地图的一致性

### 7.2 Phase 2: 双输出模式

```
INS → mapper (主输出 → car_state)
    → FG (副输出 → car_state_fg)
```

- 新增 `localization/car_state_fg` 话题
- 下游可同时订阅两路，做 A/B 对比
- 异常状态机在此阶段调参

### 7.3 Phase 3: FG 主输出

```
INS → FG (主输出 → car_state)
    → mapper (降级回退)
```

- FG 为主后端，mapper 保留为 LOST 状态回退
- 异常状态机控制切换
- `backend` 参数改为 `"factor_graph"`

---

## 8. 代码变更清单

| 文件 | 变更 | 优先级 |
|------|------|--------|
| `factor_graph_types.hpp` | 新增 `BiasState`、`AnomalyState` 枚举、颜色混淆矩阵常量 | P0 |
| `factor_graph_optimizer.hpp` | 新增 `addBiasFactor()`、`addColorTopologyFactor()`、`addGeometryPriorFactors()`；新增 `AnomalyStateMachine` 成员 | P0 |
| `factor_graph_optimizer.cpp` | 实现上述新方法；扩展 `addImuFactor()` 支持偏置；扩展 `findOrCreateLandmark()` 多维代价 | P0 |
| `location.cpp` | `feedFactorGraph()` 传入 `map_mode`；新增 FG 主输出路径；异常状态机集成 | P1 |
| `location_node.hpp` | 新增 FG 输出 publisher、anomaly state 成员 | P1 |
| `location_common.yaml` | 新增 `fg/bias_walk_sigma`、`fg/color_confusion_matrix`、`fg/anomaly/*` 参数 | P1 |

---

## 9. 与创新点的关系

| 创新点 | 本文档章节 | 详细设计文档 |
|--------|-----------|-------------|
| Innovation #1: 颜色-拓扑软关联 | §3.4, §4.1 | Task #10 |
| Innovation #2: Skidpad 双圆结构因子 | §3.5.2 | Task #11 |
| Innovation #3: 双通道重定位状态机 | §5 (RELOC 状态) | Task #12 |
