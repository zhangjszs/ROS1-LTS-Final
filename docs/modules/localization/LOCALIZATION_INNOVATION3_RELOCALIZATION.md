# 双通道重定位状态机设计 (Innovation #3)

> **版本**: v1.0
> **状态**: DRAFT
> **前置文档**: `docs/LOCALIZATION_M3_LOOP_CLOSURE_RELOC.md`
> **代码基线**: `factor_graph_optimizer.hpp` (无现有重定位实现)

---

## 1. 问题分析

### 1.1 现状

当前定位系统无任何重定位能力：

- **因子图后端**: 纯增量 iSAM2，无回环检测，无异常状态监控
- **Mapper 后端**: 纯前向建图，丢失后无法恢复
- **GNSS 依赖**: GNSS 丢失后只能靠 IMU 死算，漂移不可控

### 1.2 失效场景

| 场景 | 频率 | 影响 |
|------|------|------|
| GNSS 短暂遮挡 (< 3s) | 常见 | IMU 死算可覆盖 |
| GNSS 长时间丢失 (> 5s) | 偶发 | 位姿漂移，锥桶关联失败 |
| 锥桶长时间遮挡 | 偶发 | 无观测修正，纯 IMU 漂移 |
| 系统重启 | 罕见 | 完全丢失位姿，需冷启动 |
| 急弯后方向混淆 | 偶发 | 航向误差累积，关联错误 |

### 1.3 设计目标

- **双通道恢复**: Channel A (快速, 描述子检索) + Channel B (鲁棒, 粒子重采样)
- **自动切换**: A 失败自动降级到 B
- **有限超时**: 总重定位时间 < 5s，超时回退到安全模式
- **零人工干预**: 全自动触发、执行、验证

---

## 2. 状态机设计

### 2.1 状态定义

```
┌─────────────────────────────────────────────────────┐
│                                                     │
│  ┌──────────┐  chi²↑/GNSS↓  ┌──────────┐          │
│  │ TRACKING │──────────────►│ DEGRADED │          │
│  │          │◄──────────────│          │          │
│  └──────────┘  chi²↓/GNSS↑  └────┬─────┘          │
│       ▲                           │                 │
│       │                    match<0.2               │
│       │                    for >3s                  │
│       │                           ▼                 │
│       │                    ┌──────────┐             │
│       │                    │   LOST   │             │
│       │                    └────┬─────┘             │
│       │                         │ auto              │
│       │                    ┌────▼─────┐             │
│       │    converged       │ RELOC_A  │             │
│       │◄───────────────────│ 描述子    │             │
│       │                    └────┬─────┘             │
│       │                         │ failed            │
│       │                    ┌────▼─────┐             │
│       │    converged       │ RELOC_B  │             │
│       │◄───────────────────│ 粒子      │──► LOST    │
│       │                    └──────────┘  (timeout)  │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### 2.2 状态枚举

```cpp
enum class AnomalyState : uint8_t {
    TRACKING = 0,     // 正常跟踪
    DEGRADED = 1,     // 降级运行
    LOST = 2,         // 丢失
    RELOC_A = 3,      // Channel A: 描述子重定位
    RELOC_B = 4       // Channel B: 粒子重定位
};
```

### 2.3 状态转移表

| 当前状态 | 事件 | 目标状态 | 动作 |
|----------|------|----------|------|
| TRACKING | chi² > 50 \|\| GNSS ≤ POOR | DEGRADED | 放大协方差 ×2 |
| DEGRADED | chi² < 20 && GNSS ≥ MEDIUM | TRACKING | 恢复正常协方差 |
| DEGRADED | match_ratio < 0.2 持续 3s | LOST | 暂停 FG，切换 mapper |
| DEGRADED | 连续 10 帧无锥桶 | LOST | 同上 |
| LOST | 自动 | RELOC_A | 启动描述子检索 |
| RELOC_A | 检索成功 + 验证通过 | TRACKING | 注入回环因子 |
| RELOC_A | 检索失败 / 验证失败 | RELOC_B | 启动粒子重采样 |
| RELOC_B | 粒子收敛 | TRACKING | 注入位姿先验 |
| RELOC_B | 超时 > 5s | LOST | 等待新触发 |

---

## 3. Channel A: 描述子重定位

### 3.1 算法流程

```
输入: 当前锥桶观测 O_current, 描述子数据库 DB
输出: 重定位位姿 T_reloc 或 FAILED

1. 构建查询描述子
   D_query = buildDescriptor(O_current, dead_reckoning_pose)

2. 数据库检索
   candidates = DB.search(D_query, top_k=5, skip_recent=20)

3. 逐候选验证
   for each candidate in candidates:
       T, inliers, residual = SE2_RANSAC(O_current, candidate.landmarks)
       if inliers >= 4 AND inlier_ratio >= 0.5 AND residual < 1.0m:
           return T  // 成功

4. return FAILED
```

### 3.2 描述子构建

采用极坐标直方图编码（详见 M3 §3.2）：

```
维度: 5 rings × 12 sectors × 3 channels = 180 floats
编码: 每个 bin 中蓝/黄/其他锥桶的计数
距离: 旋转不变余弦距离
```

### 3.3 SE(2) RANSAC 验证

```cpp
struct RelocResult {
    Pose2 relative_pose;
    int inlier_count;
    double inlier_ratio;
    double mean_residual;
    bool success;
};

RelocResult SE2Ransac(
    const std::vector<Point2>& current_points,
    const std::vector<Point2>& candidate_points,
    const std::vector<uint8_t>& current_colors,
    const std::vector<uint8_t>& candidate_colors,
    int max_iterations = 50,
    double inlier_threshold = 1.5)
{
    // 1. 颜色预筛: 构建同色/NONE 匹配候选对
    auto pairs = buildColorCompatiblePairs(
        current_points, candidate_points,
        current_colors, candidate_colors);

    RelocResult best;
    best.success = false;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 2. 随机选 2 对
        auto [p1, q1, p2, q2] = randomSample(pairs, 2);

        // 3. 求解 SE(2): R, t
        auto T = solveSE2(p1, q1, p2, q2);

        // 4. 计算内点
        int inliers = 0;
        double residual_sum = 0;
        for (const auto& [pi, qi] : pairs) {
            double dist = distance(T.transform(pi), qi);
            if (dist < inlier_threshold) {
                inliers++;
                residual_sum += dist;
            }
        }

        if (inliers > best.inlier_count) {
            best.relative_pose = T;
            best.inlier_count = inliers;
            best.inlier_ratio = double(inliers) / pairs.size();
            best.mean_residual = residual_sum / std::max(1, inliers);
            best.success = (inliers >= 4 && best.inlier_ratio >= 0.5
                           && best.mean_residual < 1.0);
        }
    }

    return best;
}
```

### 3.4 性能预算

| 操作 | 目标耗时 | 说明 |
|------|----------|------|
| 描述子构建 | < 0.1 ms | 180 float 直方图 |
| 数据库检索 (200 条) | < 0.5 ms | 线性扫描 + 余弦距离 |
| SE(2) RANSAC (50 iter) | < 1.0 ms | 每 iter: 2 对点求解 + N 对验证 |
| **Channel A 总计** | < 2.0 ms | 单次尝试 |

---

## 4. Channel B: 粒子重定位

### 4.1 算法流程

```
输入: 最后已知位姿 P_last, 路标数据库 LM, 锥桶观测流
输出: 重定位位姿 T_reloc 或 TIMEOUT

1. 初始化粒子
   particles = sampleGaussian(P_last, σ_xy=5.0, σ_θ=0.5, N=100)
   如果有 GNSS: 中心 = GNSS 位置

2. 循环 (每帧锥桶观测):
   a. 预测: 用 IMU 死算更新每个粒子位姿
   b. 更新: 用锥桶观测计算每个粒子的似然权重
   c. 重采样: 如果 N_eff < N/2，执行系统重采样
   d. 收敛检查: 如果 σ_xy < 1.0m 且 σ_θ < 0.2rad 持续 5 帧
      → 返回加权均值位姿
   e. 超时检查: 如果运行 > 5.0s → TIMEOUT

3. return TIMEOUT
```

### 4.2 粒子结构

```cpp
struct Particle {
    Pose2 pose;          // (x, y, theta)
    double weight;       // 归一化权重
    double log_weight;   // 对数权重 (数值稳定)
};

class ParticleRelocator {
public:
    void Initialize(const Pose2& center, double sigma_xy, double sigma_theta,
                    int n_particles);
    void Predict(double v_forward, double wz, double dt);
    void Update(const std::vector<ConeObservation>& obs,
                const std::vector<FgLandmark>& landmarks);
    bool HasConverged() const;
    Pose2 GetPose() const;  // 加权均值

private:
    std::vector<Particle> particles_;
    int converge_count_ = 0;
    double start_time_ = 0;
};
```

### 4.3 似然计算

```cpp
void ParticleRelocator::Update(
    const std::vector<ConeObservation>& obs,
    const std::vector<FgLandmark>& landmarks)
{
    for (auto& p : particles_) {
        p.log_weight = 0.0;

        for (const auto& o : obs) {
            // 将观测投影到粒子位姿下的全局坐标
            double gx = p.pose.x()
                      + o.range * std::cos(p.pose.theta() + o.bearing);
            double gy = p.pose.y()
                      + o.range * std::sin(p.pose.theta() + o.bearing);

            // 找最近路标
            double min_dist_sq = association_radius_ * association_radius_;
            int best_lm = -1;
            for (size_t j = 0; j < landmarks.size(); ++j) {
                double dx = gx - landmarks[j].x;
                double dy = gy - landmarks[j].y;
                double d2 = dx * dx + dy * dy;
                if (d2 < min_dist_sq) {
                    min_dist_sq = d2;
                    best_lm = static_cast<int>(j);
                }
            }

            if (best_lm >= 0) {
                // 距离似然
                double dist = std::sqrt(min_dist_sq);
                p.log_weight += -0.5 * dist * dist / (sigma_particle_ * sigma_particle_);

                // 颜色加成
                if (o.color_type < 4 && landmarks[best_lm].color_type < 4 &&
                    o.color_type == landmarks[best_lm].color_type) {
                    p.log_weight += std::log(color_boost_);  // log(2.0)
                }
            } else {
                // 无匹配惩罚
                p.log_weight += std::log(miss_penalty_);  // log(0.1)
            }
        }
    }

    // 归一化权重 (log-sum-exp)
    normalizeLogWeights();
}
```

### 4.4 系统重采样

```cpp
void ParticleRelocator::resample() {
    int N = particles_.size();
    std::vector<Particle> new_particles(N);

    // 系统重采样
    double step = 1.0 / N;
    double r = uniform_random(0, step);
    double cumsum = particles_[0].weight;
    int j = 0;

    for (int i = 0; i < N; ++i) {
        double target = r + i * step;
        while (cumsum < target && j < N - 1) {
            j++;
            cumsum += particles_[j].weight;
        }
        new_particles[i] = particles_[j];
        new_particles[i].weight = 1.0 / N;

        // 添加扩散噪声防止退化
        new_particles[i].pose = Pose2(
            particles_[j].pose.x() + gaussian(0, diffuse_sigma_xy_),
            particles_[j].pose.y() + gaussian(0, diffuse_sigma_xy_),
            particles_[j].pose.theta() + gaussian(0, diffuse_sigma_theta_));
    }

    particles_ = std::move(new_particles);
}
```

---

## 5. 重定位结果注入

### 5.1 Channel A 注入 (回环因子)

```cpp
void applyRelocA(const RelocResult& result, uint64_t current_kf, uint64_t matched_kf) {
    // 回环因子: 连接当前关键帧和匹配关键帧
    auto noise = noiseModel::Diagonal::Sigmas(
        Vector3(result.mean_residual, result.mean_residual, 0.1));
    auto robust = noiseModel::Robust::Create(
        noiseModel::mEstimator::Cauchy::Create(2.0), noise);

    new_factors_->add(BetweenFactor<Pose2>(
        Symbol('x', current_kf),
        Symbol('x', matched_kf),
        result.relative_pose,
        robust));

    // 触发 iSAM2 重线性化
    isam_->update(*new_factors_, *new_values_);
    new_factors_->resize(0);
    new_values_->clear();
}
```

### 5.2 Channel B 注入 (位姿先验)

```cpp
void applyRelocB(const Pose2& reloc_pose) {
    // 强位姿先验: 将当前关键帧拉到重定位位姿
    auto noise = noiseModel::Diagonal::Sigmas(
        Vector3(1.0, 1.0, 0.2));  // 比初始先验宽松

    new_factors_->add(PriorFactor<Pose2>(
        Symbol('x', keyframe_idx_),
        reloc_pose,
        noise));

    // 触发优化
    isam_->update(*new_factors_, *new_values_);
    new_factors_->resize(0);
    new_values_->clear();
}
```

---

## 6. 诊断话题

### 6.1 新增话题: `localization/status`

| 属性 | 值 |
|------|-----|
| **话题** | `localization/status` |
| **消息类型** | `std_msgs/String` (JSON 编码) |
| **频率** | 1 Hz |

JSON 格式:

```json
{
    "timestamp": 1707580800.0,
    "anomaly_state": "TRACKING",
    "chi2_normalized": 12.5,
    "cone_match_ratio": 0.85,
    "gnss_quality": "GOOD",
    "landmark_count": 42,
    "opt_time_ms": 2.3,
    "backend": "factor_graph",
    "reloc_channel": "none",
    "covariance_scale": 1.0
}
```

### 6.2 日志输出

```cpp
// 状态转移日志
ROS_WARN("Anomaly state: %s -> %s (chi2=%.1f, match=%.2f, gnss=%s)",
         stateToString(old_state), stateToString(new_state),
         chi2_normalized, cone_match_ratio,
         gnssQualityToString(gnss_quality));

// 重定位结果日志
ROS_INFO("Relocalization %s: channel=%s, time=%.2fs, residual=%.3fm",
         success ? "SUCCESS" : "FAILED",
         channel == RELOC_A ? "descriptor" : "particle",
         elapsed_time, residual);
```

---

## 7. 配置参数汇总

### 7.1 Channel A 参数

| 参数 | 默认值 | 单位 | 配置键 |
|------|--------|------|--------|
| 子图半径 | 15.0 | m | `fg/reloc/submap_radius` |
| 描述子角度分区 | 12 | — | `fg/reloc/n_sectors` |
| 描述子距离分区 | 5 | — | `fg/reloc/n_rings` |
| 最大描述子数 | 200 | — | `fg/reloc/max_descriptors` |
| 跳过最近关键帧 | 20 | — | `fg/reloc/skip_recent` |
| 检索候选数 | 5 | — | `fg/reloc/top_k` |
| RANSAC 迭代数 | 50 | — | `fg/reloc/ransac_max_iter` |
| 内点距离阈值 | 1.5 | m | `fg/reloc/ransac_inlier_threshold` |
| 最小内点数 | 4 | — | `fg/reloc/min_inliers` |
| 最小内点比例 | 0.5 | — | `fg/reloc/min_inlier_ratio` |
| 最大残差 | 1.0 | m | `fg/reloc/max_residual` |

### 7.2 Channel B 参数

| 参数 | 默认值 | 单位 | 配置键 |
|------|--------|------|--------|
| 粒子数 | 100 | — | `fg/reloc/n_particles` |
| 初始位置 σ | 5.0 | m | `fg/reloc/particle_sigma_xy` |
| 初始航向 σ | 0.5 | rad | `fg/reloc/particle_sigma_theta` |
| 关联半径 | 3.0 | m | `fg/reloc/association_radius` |
| 观测 σ | 2.0 | m | `fg/reloc/sigma_particle` |
| 颜色加成 | 2.0 | — | `fg/reloc/color_boost` |
| 未匹配惩罚 | 0.1 | — | `fg/reloc/miss_penalty` |
| 扩散位置 σ | 0.2 | m | `fg/reloc/diffuse_sigma_xy` |
| 扩散航向 σ | 0.05 | rad | `fg/reloc/diffuse_sigma_theta` |
| 收敛位置阈值 | 1.0 | m | `fg/reloc/converge_sigma` |
| 收敛航向阈值 | 0.2 | rad | `fg/reloc/converge_yaw` |
| 收敛帧数 | 5 | — | `fg/reloc/converge_frames` |
| 超时 | 5.0 | s | `fg/reloc/timeout` |

### 7.3 状态机参数

| 参数 | 默认值 | 单位 | 配置键 |
|------|--------|------|--------|
| chi² 降级阈值 | 50.0 | — | `fg/anomaly/chi2_degrade` |
| chi² 恢复阈值 | 20.0 | — | `fg/anomaly/chi2_recover` |
| chi² 窗口 | 5 | 帧 | `fg/anomaly/chi2_window` |
| 匹配率丢失阈值 | 0.2 | — | `fg/anomaly/match_ratio_lost` |
| 丢失持续时间 | 3.0 | s | `fg/anomaly/match_lost_duration` |
| 无锥桶最大帧数 | 10 | 帧 | `fg/anomaly/no_cone_frames_max` |
| 协方差降级倍数 | 2.0 | — | `fg/anomaly/cov_degrade_scale` |
| 协方差丢失倍数 | 5.0 | — | `fg/anomaly/cov_lost_scale` |

---

## 8. 实验设计

### 8.1 遮挡注入实验

| 实验 | 遮挡时长 | 预期 Channel A | 预期 Channel B |
|------|----------|---------------|---------------|
| 短遮挡 | 5s | 成功 (有历史描述子) | 不触发 |
| 中遮挡 | 10s | 可能成功 | 备用 |
| 长遮挡 | 15s | 可能失败 (漂移过大) | 成功 (粒子扩散) |

### 8.2 重启实验

| 实验 | 条件 | 预期 |
|------|------|------|
| 热重启 | 保留路标数据库 | Channel A 快速恢复 |
| 冷重启 | 清空所有状态 | Channel B + GNSS 恢复 |
| 无 GNSS 冷重启 | 清空状态 + GNSS INVALID | Channel B 扩大搜索范围 |

### 8.3 指标

| 指标 | 通过 | 失败 |
|------|------|------|
| Channel A 成功率 | > 80% (有历史数据时) | < 50% |
| Channel B 成功率 | > 90% (有路标数据时) | < 70% |
| 平均恢复时间 | < 2.0s | > 5.0s |
| 最大横向偏差 (LOST 期间) | < 2.0m | > 5.0m |
| 误重定位率 | < 5% | > 10% |

---

## 9. 风险与缓解

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 描述子区分度不足 | Channel A 误匹配 | 增加 sector/ring 分辨率 |
| 粒子退化 | Channel B 不收敛 | 系统重采样 + 扩散噪声 |
| 状态机振荡 | 频繁切换 TRACKING↔DEGRADED | 添加滞后 (hysteresis) |
| 重定位后残差突增 | iSAM2 不稳定 | 使用鲁棒核 (Cauchy) 注入 |
| 计算超预算 | 实时性不满足 | 限制粒子数和 RANSAC 迭代 |

---

## 10. 代码变更清单

| 文件 | 变更 | 优先级 |
|------|------|--------|
| `anomaly_state_machine.hpp/cpp` | 新文件: 完整状态机实现 | P0 |
| `descriptor_relocator.hpp/cpp` | 新文件: Channel A 描述子检索 | P0 |
| `particle_relocator.hpp/cpp` | 新文件: Channel B 粒子重采样 | P0 |
| `factor_graph_types.hpp` | 新增 `AnomalyState`、`SubMapDescriptor`、`RelocConfig`、`Particle` | P0 |
| `factor_graph_optimizer.hpp` | 新增状态机/重定位器成员、chi² 计算方法 | P0 |
| `factor_graph_optimizer.cpp` | 集成状态机到 `TryUpdate()`、回环/先验因子注入 | P0 |
| `location.cpp` | 状态机评估、诊断话题发布、降级行为切换 | P1 |
| `location_node.hpp` | 新增诊断 publisher、`AnomalyState` 成员 | P1 |
| `location_common.yaml` | 新增 `fg/reloc/*`、`fg/anomaly/*` 全部参数 | P1 |

---

## 11. 与其他文档的关系

| 文档 | 关系 |
|------|------|
| M2 CTF-Graph 设计 §5 | 异常状态机的概要设计，本文档为详细展开 |
| M3 回环与重定位 | 本文档是 M3 中 Channel A/B 的完整实现设计 |
| Innovation #1 颜色因子 | 颜色混淆矩阵用于粒子似然中的颜色加成 |
| Innovation #2 Skidpad 圆 | 双圆约束用于 Skidpad 交叉区域的方向感知 |
| M4 验证计划 | 重定位实验设计与 M4 场景 S8 对应 |
| M5 风险清单 | F10 (无重定位) 的完整解决方案 |
