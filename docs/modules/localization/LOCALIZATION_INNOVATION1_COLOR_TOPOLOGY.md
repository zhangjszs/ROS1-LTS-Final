# 颜色-拓扑软关联因子设计 (Innovation #1)

> **版本**: v1.0
> **状态**: DRAFT
> **前置文档**: `docs/LOCALIZATION_M2_CTF_GRAPH_DESIGN.md` §3.4, §4.1
> **代码基线**: `factor_graph_optimizer.cpp:305` (`addConeFactors`)

---

## 1. 问题分析

### 1.1 现状

当前因子图后端的颜色处理（`addConeFactors`, line 346-349）：

```cpp
// 现有实现: 硬惩罚
if (obs.color_type != 4 && lm.color_type != 4 &&
    obs.color_type != lm.color_type) {
    sr += cfg_.color_mismatch_penalty * cfg_.color_weight;
}
```

问题：
- **二值化**: 颜色匹配/不匹配只有两种状态，无法表达部分置信
- **NONE 被忽略**: 当观测或路标颜色为 NONE 时不施加任何约束
- **无拓扑信息**: 未利用 FSG 规则中"左蓝右黄"的赛道拓扑

### 1.2 LiDAR 颜色分类特性

纯 LiDAR 颜色分类（无相机融合）的典型表现：

| 距离范围 | 蓝/黄区分率 | NONE 比例 | 橙色误判率 |
|----------|------------|-----------|-----------|
| 0 - 5m | ~85% | ~5% | ~5% |
| 5 - 15m | ~60% | ~25% | ~10% |
| 15 - 30m | ~30% | ~55% | ~15% |

远距离锥桶大部分返回 NONE 或被误分类，硬门控会导致大量有效观测被丢弃或错误关联。

---

## 2. 颜色混淆矩阵

### 2.1 矩阵定义

`C[observed][true]` 表示观测到颜色 `observed` 时，真实颜色为 `true` 的概率：

```
              true: BLUE   YELLOW  ORANGE  NONE
obs: BLUE          0.90    0.05    0.03    0.02
     YELLOW        0.05    0.90    0.03    0.02
     ORANGE        0.10    0.10    0.75    0.05
     NONE          0.25    0.25    0.25    0.25
```

### 2.2 矩阵解读

- **对角线**: 正确分类概率（蓝/黄 90%，橙 75%，NONE 均匀分布）
- **蓝↔黄互混**: 5%（低概率但存在）
- **NONE 行**: 均匀分布（无信息，不偏向任何颜色）
- **NONE 列**: 低概率（真实颜色为 NONE 的锥桶极少）

### 2.3 距离自适应（可选扩展）

远距离颜色分类更不可靠，可引入距离衰减：

```
C_adaptive[obs][true](range) = C_base[obs][true]^(decay(range))
decay(range) = max(0.3, 1.0 - range / range_max)

当 range → range_max 时，矩阵趋向均匀分布（无信息）
当 range → 0 时，矩阵保持原始值（高置信）
```

初期实现使用固定矩阵，距离自适应作为后续优化。

---

## 3. 颜色软权重因子

### 3.1 颜色代价计算

对每个观测-路标关联 `(o_i, l_j)`：

```
color_cost(i, j) = -log(C[obs_color_i][map_color_j])
```

代价值示例：

| 观测 \ 路标 | BLUE | YELLOW | ORANGE | NONE |
|-------------|------|--------|--------|------|
| BLUE | 0.105 | 2.996 | 3.507 | 3.912 |
| YELLOW | 2.996 | 0.105 | 3.507 | 3.912 |
| ORANGE | 2.303 | 2.303 | 0.288 | 2.996 |
| NONE | 1.386 | 1.386 | 1.386 | 1.386 |

### 3.2 转化为噪声模型增量

将颜色代价转化为等效的 range σ 增量：

```
σ_color = sqrt(2 × color_cost) × color_weight
σ_range_total = σ_range_base + σ_range_scale × range + σ_color
```

| 场景 | color_cost | σ_color (weight=0.5) |
|------|-----------|---------------------|
| 蓝观测-蓝路标 | 0.105 | 0.229 m |
| 蓝观测-黄路标 | 2.996 | 1.224 m |
| NONE 观测-任意 | 1.386 | 0.833 m |

效果：颜色匹配时几乎不增加噪声，颜色冲突时显著放大噪声（降低因子权重），
NONE 观测时适度放大噪声（保留但降权）。

---

## 4. 拓扑一致性约束

### 4.1 FSG 赛道拓扑规则

- 赛道左侧: 蓝色锥桶
- 赛道右侧: 黄色锥桶
- 起/终点线: 橙色大锥桶
- 赛道宽度: 3.0 - 5.0m（FSG 规则）

### 4.2 拓扑关系判定

对已关联的路标对 `(l_j, l_k)`，判定拓扑关系：

```cpp
enum class TopoRelation { SAME_SIDE, OPPOSITE_SIDE, UNKNOWN };

TopoRelation ClassifyRelation(const FgLandmark& lj, const FgLandmark& lk,
                               const Pose2& vehicle_pose) {
    // 将路标转换到车体坐标系
    double ly_j = toBodyFrame(lj, vehicle_pose).y;
    double ly_k = toBodyFrame(lk, vehicle_pose).y;

    if (ly_j * ly_k > 0)  // 同侧
        return TopoRelation::SAME_SIDE;
    else if (std::abs(ly_j - ly_k) > track_width * 0.5)  // 对侧
        return TopoRelation::OPPOSITE_SIDE;
    else
        return TopoRelation::UNKNOWN;
}
```

### 4.3 拓扑代价

```
topo_cost(j, k) =
    0.0                  如果关系 = UNKNOWN
    0.0                  如果 SAME_SIDE 且颜色一致 (蓝-蓝 或 黄-黄)
    0.0                  如果 OPPOSITE_SIDE 且颜色互补 (蓝-黄)
    topo_penalty (2.0)   如果违反上述规则
```

### 4.4 拓扑约束集成

拓扑代价叠加到数据关联的多维代价函数中：

```
total_cost(i, j) = w_maha × d_maha(i, j)²
                 + w_color × color_cost(i, j)
                 + w_topo × Σ_k topo_cost(j, k)  // k 为 j 的相邻路标
```

相邻路标定义: 与 `l_j` 距离 < `neighbor_radius` (5.0m) 的已有路标。

---

## 5. 代码实现方案

### 5.1 新增方法: `addColorTopologyFactor()`

```cpp
void FactorGraphOptimizer::addColorTopologyFactor(
    int obs_idx, int landmark_idx,
    double& sigma_range_out)
{
    const auto& obs = cone_obs_[obs_idx];
    const auto& lm = landmarks_[landmark_idx];

    // 1. 颜色软权重
    double color_cost = -std::log(
        color_confusion_matrix_[obs.color_type][lm.color_type]);
    double sigma_color = std::sqrt(2.0 * color_cost) * cfg_.color_weight;
    sigma_range_out += sigma_color;

    // 2. 拓扑一致性 (仅当路标有已知颜色时)
    if (lm.color_type < 4) {  // 非 NONE
        double topo_penalty_sum = 0.0;
        int neighbor_count = 0;
        for (const auto& other_lm : landmarks_) {
            if (other_lm.id == lm.id) continue;
            double dist = std::hypot(lm.x - other_lm.x, lm.y - other_lm.y);
            if (dist > cfg_.neighbor_radius) continue;
            if (other_lm.color_type >= 4) continue;  // 跳过 NONE

            auto relation = classifyRelation(lm, other_lm);
            if (relation == TopoRelation::SAME_SIDE &&
                lm.color_type != other_lm.color_type) {
                topo_penalty_sum += cfg_.topo_penalty;
            } else if (relation == TopoRelation::OPPOSITE_SIDE &&
                       !isComplementary(lm.color_type, other_lm.color_type)) {
                topo_penalty_sum += cfg_.topo_penalty;
            }
            neighbor_count++;
        }
        if (neighbor_count > 0) {
            sigma_range_out += (topo_penalty_sum / neighbor_count)
                             * cfg_.topo_weight;
        }
    }
}
```

### 5.2 修改 `addConeFactors()`

在现有 `addConeFactors()` 中替换硬惩罚逻辑：

```cpp
// 替换前 (lines 346-349):
if (obs.color_type != 4 && lm.color_type != 4 &&
    obs.color_type != lm.color_type) {
    sr += cfg_.color_mismatch_penalty * cfg_.color_weight;
}

// 替换后:
addColorTopologyFactor(i, lm_idx, sr);
```

### 5.3 修改 `findOrCreateLandmark()`

在数据关联中使用多维代价替代纯欧氏距离：

```cpp
int FactorGraphOptimizer::findOrCreateLandmark(
    const ConeObservation& obs, const Pose2& pose)
{
    double gx = pose.x() + obs.range * std::cos(pose.theta() + obs.bearing);
    double gy = pose.y() + obs.range * std::sin(pose.theta() + obs.bearing);

    int best_idx = -1;
    double best_cost = cfg_.gate_threshold;  // 替代 merge_distance

    for (size_t j = 0; j < landmarks_.size(); ++j) {
        double dx = gx - landmarks_[j].x;
        double dy = gy - landmarks_[j].y;

        // Mahalanobis 距离
        double d_maha_sq = dx * dx + dy * dy;  // 简化: 各向同性

        // 颜色代价
        double c_cost = -std::log(
            color_confusion_matrix_[obs.color_type][landmarks_[j].color_type]);

        // 拓扑代价 (简化: 仅检查颜色侧向一致性)
        double t_cost = 0.0;
        // ... 拓扑检查逻辑

        double total_cost = cfg_.w_maha * d_maha_sq
                          + cfg_.w_color * c_cost
                          + cfg_.w_topo * t_cost;

        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_idx = static_cast<int>(j);
        }
    }

    if (best_idx >= 0) {
        // 更新已有路标
        landmarks_[best_idx].obs_count++;
        if (landmarks_[best_idx].color_type == 4 && obs.color_type != 4)
            landmarks_[best_idx].color_type = obs.color_type;
        return best_idx;
    }

    // 创建新路标 (同现有逻辑)
    // ...
}
```

---

## 6. 新增配置参数

| 参数 | 默认值 | 单位 | 配置键 |
|------|--------|------|--------|
| 颜色混淆矩阵 | 见 §2.1 | — | `fg/color_confusion_matrix` |
| 颜色权重 | 0.5 | — | `fg/w_color` |
| 拓扑权重 | 0.3 | — | `fg/w_topo` |
| Mahalanobis 权重 | 1.0 | — | `fg/w_maha` |
| 拓扑惩罚 | 2.0 | — | `fg/topo_penalty` |
| 相邻路标半径 | 5.0 | m | `fg/neighbor_radius` |
| 关联门限 | 4.0 | — | `fg/gate_threshold` |
| 距离自适应衰减 | false | — | `fg/color_range_adaptive` |
| 最大衰减距离 | 30.0 | m | `fg/color_range_max` |

---

## 7. 实验设计

### 7.1 消融实验

| 实验 | 颜色软权重 | 拓扑约束 | 预期效果 |
|------|-----------|---------|---------|
| Baseline | 关 (硬惩罚) | 关 | 当前性能 |
| A: 仅颜色 | 开 | 关 | 远距离关联改善 |
| B: 仅拓扑 | 关 (硬惩罚) | 开 | 侧向一致性改善 |
| C: 颜色+拓扑 | 开 | 开 | 综合最优 |

### 7.2 鲁棒性测试

| 测试 | 方法 | 指标 |
|------|------|------|
| 颜色扰动 10% | 随机翻转 10% 颜色 | `association_accuracy`, `position_rms_m` |
| 颜色扰动 20% | 随机翻转 20% 颜色 | 同上 |
| 颜色扰动 50% | 随机翻转 50% 颜色 | 同上 |
| 全 NONE | 所有颜色设为 NONE | 同上（应退化到纯几何关联） |
| 锥桶遮挡 30% | 随机移除 30% 检测 | `position_rms_m`, `map_consistency` |

### 7.3 预期结果

- 颜色扰动 10%: `association_accuracy` 下降 < 5%（软权重吸收噪声）
- 颜色扰动 50%: `association_accuracy` 下降 < 20%（优于硬门控的 > 40%）
- 全 NONE: 性能与无颜色 baseline 一致（NONE 行均匀分布，不引入偏差）
- 正常场景: `position_rms_m` 改善 5-15%（更准确的关联）

---

## 8. 风险与缓解

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 混淆矩阵不准确 | 颜色权重偏差 | 从实际 bag 统计校准矩阵 |
| 拓扑判定错误 | 错误惩罚正确关联 | 仅对高置信路标施加拓扑约束 |
| 计算开销增加 | 超出实时预算 | 限制相邻路标搜索范围 |
| 参数过多 | 调参困难 | 提供保守默认值，逐步调优 |

---

## 9. 代码变更清单

| 文件 | 变更 | 优先级 |
|------|------|--------|
| `factor_graph_types.hpp` | 新增 `ColorConfusionMatrix` 类型别名、`TopoRelation` 枚举、关联权重参数 | P0 |
| `factor_graph_optimizer.hpp` | 新增 `addColorTopologyFactor()` 声明、`classifyRelation()` 声明 | P0 |
| `factor_graph_optimizer.cpp` | 实现颜色软权重、拓扑一致性检查、多维代价关联 | P0 |
| `location_common.yaml` | 新增 `fg/color_confusion_matrix`、`fg/w_color`、`fg/w_topo` 等参数 | P1 |
| `location.cpp` | `feedFactorGraphCones()` 传递颜色信息（已有） | — |
