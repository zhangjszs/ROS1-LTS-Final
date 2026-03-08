# 回环检测与重定位设计规格 (Loop Closure & Relocalization)

> **版本**: v1.0
> **状态**: DRAFT
> **前置文档**: `docs/LOCALIZATION_M2_CTF_GRAPH_DESIGN.md`
> **代码基线**: `localization_core/factor_graph_optimizer.hpp` (shadow mode)

---

## 1. 设计目标

当前因子图后端（shadow mode）为纯增量 iSAM2，无回环检测和重定位能力。
当 GNSS 信号丢失、锥桶长时间遮挡或系统重启后，无法恢复全局位姿。

本文档设计：

- **回环检测**: 基于锥桶子图描述子的场景识别
- **重定位**: 双通道恢复策略（描述子检索 + 粒子重采样）
- **状态机**: TRACKING → DEGRADED → LOST → RELOCATING_A → RELOCATING_B → TRACKING
- **Skidpad 特殊处理**: 交叉区域回环与圈数计数

---

## 2. 触发条件

### 2.1 降级触发 (TRACKING → DEGRADED)

满足任一条件即触发：

| 条件 | 阈值 | 配置键 |
|------|------|--------|
| 最近 5 帧平均归一化 chi² | > `chi2_degrade_threshold` (50.0) | `fg/anomaly/chi2_degrade` |
| GNSS 质量 | 降为 POOR 或 INVALID | — |
| 锥桶匹配率 | < 0.5 持续 > 1.0s | `fg/anomaly/match_ratio_warn` |

### 2.2 丢失触发 (DEGRADED → LOST)

满足任一条件即触发：

| 条件 | 阈值 | 配置键 |
|------|------|--------|
| 锥桶匹配率 | < 0.2 持续 > 3.0s | `fg/anomaly/match_ratio_lost` |
| 连续无有效锥桶关联帧数 | > 10 帧 | `fg/anomaly/no_cone_frames_max` |
| 位姿跳变 | 单帧位移 > 5.0m | `fg/anomaly/pose_jump_threshold` |

### 2.3 恢复触发 (DEGRADED → TRACKING)

同时满足：

- 平均 chi² < `chi2_recover_threshold` (20.0)
- GNSS 质量 ≥ MEDIUM
- 锥桶匹配率 > 0.5 持续 > 1.0s

---

## 3. 锥桶子图描述子 (Cone-Submap Descriptor)

### 3.1 子图定义

以每个关键帧为中心，收集半径 `submap_radius` (15.0m) 内的路标，
构建局部锥桶子图 `S_k = {(Δx_j, Δy_j, color_j) | j ∈ landmarks_near_k}`。

坐标 `(Δx, Δy)` 为路标相对于关键帧位姿的局部坐标（旋转对齐到关键帧航向）。

### 3.2 描述子编码

采用简化的 Scan Context 思路，将局部子图编码为极坐标直方图：

```
角度分区: N_sector = 12 (每 30°)
距离分区: N_ring = 5 (0-3m, 3-6m, 6-9m, 9-12m, 12-15m)
通道: 3 (蓝色计数, 黄色计数, 其他计数)

descriptor[ring][sector][channel] = count_of_cones_in_bin
```

描述子维度: `5 × 12 × 3 = 180` (float32)

### 3.3 描述子距离

```
dist(D_a, D_b) = min_over_rotation_shifts(
    cosine_distance(flatten(D_a), flatten(rotate_sectors(D_b, shift)))
)
```

旋转对齐: 遍历 12 个 sector 偏移，取最小余弦距离。

### 3.4 描述子数据库

```cpp
struct SubMapDescriptor {
    uint64_t keyframe_id;
    Pose2 keyframe_pose;
    std::array<float, 180> descriptor;
    std::vector<FgLandmark> local_landmarks;  // 用于验证
};

// 数据库: 按关键帧 ID 索引
std::vector<SubMapDescriptor> descriptor_db_;
```

每创建一个关键帧，计算并存储描述子。数据库大小上限 `max_descriptors` (200)，
超出时移除最早的描述子。

---

## 4. 回环检测 (Channel A: 描述子检索)

### 4.1 检索流程

```
1. 构建当前帧描述子 D_query
2. 在数据库中搜索 top-K 候选 (K=5)
   - 排除最近 N_skip (20) 个关键帧（避免短期自匹配）
   - 按描述子距离排序
3. 对每个候选 D_candidate:
   a. 提取候选关键帧的局部路标集
   b. 与当前观测路标做 SE(2) RANSAC 匹配
   c. 计算内点数和残差
4. 接受条件:
   - 内点数 ≥ min_inliers (4)
   - 内点比例 ≥ min_inlier_ratio (0.5)
   - 平均残差 < max_residual (1.0m)
```

### 4.2 SE(2) RANSAC

```
输入: 当前路标集 P = {p_i}, 候选路标集 Q = {q_j}
颜色预筛: 仅匹配同色或 NONE 的路标对

RANSAC 迭代 (max_iterations = 50):
  1. 随机选 2 对匹配点
  2. 求解 SE(2) 变换 T = [R|t]
  3. 将 T 应用到 P，计算与 Q 的最近邻距离
  4. 统计内点 (距离 < inlier_threshold = 1.5m)
  5. 保留最大内点集的 T

输出: T_best, inliers, residual
```

### 4.3 回环因子注入

验证通过后，向因子图注入回环因子：

```cpp
// 回环因子: BetweenFactor<Pose2>
// 连接当前关键帧 x_current 和匹配关键帧 x_matched
Pose2 relative_pose = T_best;  // RANSAC 求解的相对位姿
auto noise = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));
auto robust = noiseModel::Robust::Create(
    noiseModel::mEstimator::Cauchy::Create(2.0), noise);
new_factors_->add(BetweenFactor<Pose2>(
    Symbol('x', current_kf), Symbol('x', matched_kf), relative_pose, robust));
```

---

## 5. 粒子重采样重定位 (Channel B: 备用)

### 5.1 触发条件

Channel A 失败（无候选或所有候选验证失败）时启动 Channel B。

### 5.2 粒子初始化

```
N_particles = 100
分布: 以最后已知位姿为中心的高斯分布
  σ_xy = 5.0m
  σ_θ = 0.5 rad

如果有 GNSS (即使 POOR 质量):
  中心 = GNSS 位置
  σ_xy = gnss_sigma (10.0m for POOR)
```

### 5.3 粒子权重更新

每收到一帧锥桶观测，更新粒子权重：

```
for each particle p_i:
    weight_i = 1.0
    for each cone observation o_k:
        # 将观测投影到粒子位姿下的全局坐标
        g_k = transform(p_i.pose, o_k.range, o_k.bearing)
        # 在路标数据库中找最近路标
        nearest = find_nearest_landmark(g_k)
        if nearest.dist < association_radius (3.0m):
            likelihood = exp(-0.5 * nearest.dist² / σ_particle²)
            if color_match(o_k.color, nearest.color):
                likelihood *= color_boost (2.0)
            weight_i *= likelihood
        else:
            weight_i *= miss_penalty (0.1)

    normalize(weights)
```

### 5.4 重采样与收敛

```
每帧执行:
  1. 权重更新
  2. 有效粒子数 N_eff = 1 / Σ(w_i²)
  3. 如果 N_eff < N_particles / 2: 系统重采样
  4. 粒子扩散: 加入小噪声防止退化

收敛判定:
  - 粒子位置标准差 < converge_sigma (1.0m)
  - 粒子航向标准差 < converge_yaw (0.2 rad)
  - 持续 > converge_frames (5) 帧

收敛后:
  - 取加权均值位姿作为重定位结果
  - 注入 PriorFactor<Pose2> 到因子图
  - 转移到 TRACKING 状态
```

### 5.5 超时处理

```
如果 Channel B 运行 > reloc_timeout (5.0s) 未收敛:
  - 回退到 LOST 状态
  - 继续使用 mapper 后端输出
  - 等待新的触发条件重试
```

---

## 6. 状态机详细设计

### 6.1 状态定义

```
┌───────────┐   chi²↑ / GNSS lost   ┌───────────┐
│ TRACKING  │ ──────────────────────►│ DEGRADED  │
│           │◄────────────────────── │           │
└───────────┘   chi²↓ & GNSS back   └─────┬─────┘
                                           │ cone_match < 0.2
                                           │ for > 3s
                                           ▼
                                     ┌───────────┐
                                     │   LOST    │
                                     └─────┬─────┘
                                           │ auto trigger
                                     ┌─────▼─────┐
                                     │  RELOC_A  │──► TRACKING (converged)
                                     │ (描述子)   │
                                     └─────┬─────┘
                                           │ A failed
                                     ┌─────▼─────┐
                                     │  RELOC_B  │──► TRACKING (converged)
                                     │ (粒子)     │──► LOST (timeout)
                                     └───────────┘
```

### 6.2 各状态行为

| 状态 | FG 优化 | 输出源 | 协方差 | 诊断标记 |
|------|---------|--------|--------|----------|
| TRACKING | 正常运行 | FG 位姿 | 正常 | `quality=GOOD` |
| DEGRADED | 正常运行 | FG 位姿 | ×2.0 放大 | `quality=DEGRADED` |
| LOST | 暂停 | mapper 后端 | ×5.0 放大 | `quality=LOST` |
| RELOC_A | 暂停 | mapper 后端 | ×5.0 放大 | `quality=RELOCATING` |
| RELOC_B | 暂停 | mapper 后端 | ×5.0 放大 | `quality=RELOCATING` |

### 6.3 状态转移伪代码

```cpp
void AnomalyStateMachine::Evaluate(
    double chi2_normalized,
    double cone_match_ratio,
    GnssQuality gnss_quality,
    double timestamp)
{
    switch (state_) {
    case TRACKING:
        chi2_history_.push(chi2_normalized);
        if (chi2_history_.mean() > chi2_degrade_threshold_ ||
            gnss_quality <= GnssQuality::POOR) {
            TransitionTo(DEGRADED, timestamp);
        }
        break;

    case DEGRADED:
        chi2_history_.push(chi2_normalized);
        if (chi2_history_.mean() < chi2_recover_threshold_ &&
            gnss_quality >= GnssQuality::MEDIUM &&
            cone_match_ratio > 0.5) {
            TransitionTo(TRACKING, timestamp);
        } else if (cone_match_ratio < 0.2 &&
                   (timestamp - low_match_start_) > 3.0) {
            TransitionTo(LOST, timestamp);
        } else if (no_cone_frames_ > 10) {
            TransitionTo(LOST, timestamp);
        }
        break;

    case LOST:
        StartRelocalization(timestamp);
        TransitionTo(RELOC_A, timestamp);
        break;

    case RELOC_A:
        if (descriptor_reloc_.HasConverged()) {
            ApplyRelocResult(descriptor_reloc_.GetPose());
            TransitionTo(TRACKING, timestamp);
        } else if (descriptor_reloc_.HasFailed()) {
            TransitionTo(RELOC_B, timestamp);
            particle_reloc_.Initialize(last_known_pose_, landmarks_);
        }
        break;

    case RELOC_B:
        particle_reloc_.Update(cone_observations_);
        if (particle_reloc_.HasConverged()) {
            ApplyRelocResult(particle_reloc_.GetPose());
            TransitionTo(TRACKING, timestamp);
        } else if (timestamp - reloc_start_ > reloc_timeout_) {
            TransitionTo(LOST, timestamp);
        }
        break;
    }
}
```

---

## 7. Skidpad 特殊处理

### 7.1 交叉区域回环

Skidpad 8 字形赛道在中心有交叉区域，车辆每圈经过两次。
这是天然的回环机会，但也是关联歧义最大的区域。

**处理策略**:

1. **交叉区域检测**: 当车辆位置距两圆心连线中点 < `crossover_radius` (3.0m) 时，
   标记进入交叉区域。

2. **方向感知关联**: 在交叉区域内，数据关联额外考虑车辆航向：
   ```
   heading_cost = |angle_diff(vehicle_heading, landmark_approach_heading)|
   ```
   同一路标从不同方向接近时，`approach_heading` 不同，避免错误关联。

3. **回环约束**: 每次通过交叉区域时，与上一次通过的关键帧建立回环因子，
   约束累积漂移。

### 7.2 圈数计数

```
lap_count = 0
last_crossover_direction = NONE  // LEFT_CIRCLE or RIGHT_CIRCLE

on_crossover_exit(direction):
    if direction != last_crossover_direction:
        // 切换了圆，半圈完成
        half_lap_count++
    last_crossover_direction = direction
    if half_lap_count >= 2:
        lap_count++
        half_lap_count = 0
```

Skidpad 赛事要求完成 4 圈（2 圈左 + 2 圈右），圈数计数用于终止判断。

---

## 8. 配置参数

### 8.1 新增参数表

| 参数 | 默认值 | 单位 | 配置键 |
|------|--------|------|--------|
| 子图半径 | 15.0 | m | `fg/reloc/submap_radius` |
| 描述子角度分区 | 12 | — | `fg/reloc/n_sectors` |
| 描述子距离分区 | 5 | — | `fg/reloc/n_rings` |
| 最大描述子数 | 200 | — | `fg/reloc/max_descriptors` |
| 跳过最近关键帧数 | 20 | — | `fg/reloc/skip_recent` |
| 检索候选数 K | 5 | — | `fg/reloc/top_k` |
| RANSAC 最大迭代 | 50 | — | `fg/reloc/ransac_max_iter` |
| RANSAC 内点阈值 | 1.5 | m | `fg/reloc/ransac_inlier_threshold` |
| 最小内点数 | 4 | — | `fg/reloc/min_inliers` |
| 最小内点比例 | 0.5 | — | `fg/reloc/min_inlier_ratio` |
| 最大残差 | 1.0 | m | `fg/reloc/max_residual` |
| 粒子数 | 100 | — | `fg/reloc/n_particles` |
| 粒子位置标准差 | 5.0 | m | `fg/reloc/particle_sigma_xy` |
| 粒子航向标准差 | 0.5 | rad | `fg/reloc/particle_sigma_theta` |
| 关联半径 | 3.0 | m | `fg/reloc/association_radius` |
| 收敛位置阈值 | 1.0 | m | `fg/reloc/converge_sigma` |
| 收敛航向阈值 | 0.2 | rad | `fg/reloc/converge_yaw` |
| 收敛帧数 | 5 | — | `fg/reloc/converge_frames` |
| 重定位超时 | 5.0 | s | `fg/reloc/timeout` |
| 交叉区域半径 | 3.0 | m | `fg/reloc/crossover_radius` |

### 8.2 异常状态机参数

| 参数 | 默认值 | 单位 | 配置键 |
|------|--------|------|--------|
| chi² 降级阈值 | 50.0 | — | `fg/anomaly/chi2_degrade` |
| chi² 恢复阈值 | 20.0 | — | `fg/anomaly/chi2_recover` |
| chi² 滑动窗口 | 5 | 帧 | `fg/anomaly/chi2_window` |
| 匹配率丢失阈值 | 0.2 | — | `fg/anomaly/match_ratio_lost` |
| 匹配率丢失持续时间 | 3.0 | s | `fg/anomaly/match_lost_duration` |
| 无锥桶最大帧数 | 10 | 帧 | `fg/anomaly/no_cone_frames_max` |
| 位姿跳变阈值 | 5.0 | m | `fg/anomaly/pose_jump_threshold` |
| 协方差降级倍数 | 2.0 | — | `fg/anomaly/cov_degrade_scale` |
| 协方差丢失倍数 | 5.0 | — | `fg/anomaly/cov_lost_scale` |

---

## 9. 代码变更清单

| 文件 | 变更 | 优先级 |
|------|------|--------|
| `factor_graph_types.hpp` | 新增 `AnomalyState` 枚举、`SubMapDescriptor` 结构体、`RelocConfig` 结构体 | P0 |
| `factor_graph_optimizer.hpp` | 新增 `AnomalyStateMachine` 类声明、`DescriptorRelocator` 类声明、`ParticleRelocator` 类声明 | P0 |
| `factor_graph_optimizer.cpp` | 实现 chi² 计算、描述子构建/检索、RANSAC 验证、粒子重采样、回环因子注入 | P0 |
| `anomaly_state_machine.hpp/cpp` | 新文件：状态机完整实现 | P0 |
| `descriptor_relocator.hpp/cpp` | 新文件：Channel A 描述子检索实现 | P1 |
| `particle_relocator.hpp/cpp` | 新文件：Channel B 粒子重采样实现 | P1 |
| `location.cpp` | 集成状态机到 `feedFactorGraph()`、新增诊断话题发布 | P1 |
| `location_node.hpp` | 新增 `anomaly_state_` 成员、诊断 publisher | P1 |
| `location_common.yaml` | 新增 `fg/anomaly/*` 和 `fg/reloc/*` 参数 | P1 |

---

## 10. 与其他文档的关系

| 文档 | 关系 |
|------|------|
| M1 接口冻结 | 新增 `localization/status` 诊断话题（非 breaking change） |
| M2 CTF-Graph 设计 | §5 异常状态机的详细展开；§3.4 颜色因子用于重定位验证 |
| Innovation #1 | 颜色混淆矩阵用于粒子权重中的颜色匹配 |
| Innovation #2 | Skidpad 双圆约束用于交叉区域回环检测 |
| Innovation #3 | 本文档即为 Innovation #3 的完整设计 |
