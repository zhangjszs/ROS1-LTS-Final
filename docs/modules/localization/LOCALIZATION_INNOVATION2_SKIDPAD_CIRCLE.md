# Skidpad 双圆结构因子设计 (Innovation #2)

> **版本**: v1.0
> **状态**: DRAFT
> **前置文档**: `docs/LOCALIZATION_M2_CTF_GRAPH_DESIGN.md` §3.5.2
> **代码基线**: `location_mapper.cpp:611` (`passesGeometryFilter`)

---

## 1. 问题分析

### 1.1 Skidpad 赛道几何

FSG 规则定义的 Skidpad 赛道为标准 8 字形：

```
        ← 18.25m →
    ┌───────┐   ┌───────┐
   /  左圆   \ /  右圆   \
  │  R=15.25m ╳ R=15.25m  │
   \         / \         /
    └───────┘   └───────┘
         交叉区域
```

- 内圆半径: R = 15.25m（锥桶排列半径）
- 两圆心距: D = 18.25m
- 赛道宽度: W = 3.0m
- 锥桶排列: 内圈 + 外圈，间距约 5m

### 1.2 现有实现

`passesGeometryFilter()` (location_mapper.cpp:621-644) 中的 skidpad 验证：

```cpp
// 现有实现: 硬门控
if (enable_circle_validation) {
    double half_dist = circle_center_dist / 2.0;
    double d1 = std::hypot(lx, ly - half_dist);
    double d2 = std::hypot(lx, ly + half_dist);
    double min_err = std::min(std::abs(d1 - R), std::abs(d2 - R));
    if (min_err > circle_tolerance) {
        return false;  // 硬拒绝
    }
}
```

问题：
- **仅用于过滤**: 几何约束只在 mapper 入图时使用，不参与因子图优化
- **硬门控**: 超出容差即丢弃，无法利用部分信息
- **无结构约束**: 因子图不知道锥桶应该排列在圆上
- **交叉区域**: 两圆重叠区域的锥桶关联容易混淆

### 1.3 设计目标

将 Skidpad 双圆几何作为因子图中的结构化先验：
- 路标应位于两个已知圆的圆周附近
- 圆心位置可作为优化变量（适应实际赛道偏差）
- 交叉区域需要特殊处理

---

## 2. 双圆结构因子

### 2.1 因子定义

对每个锥桶路标 `l_j`，添加一元因子约束其到最近圆周的距离：

```
residual(l_j) = min(|dist(l_j, c1) - R|, |dist(l_j, c2) - R|)
```

其中：
- `c1 = (0, +D/2)` = 左圆圆心（base_link 初始系）
- `c2 = (0, -D/2)` = 右圆圆心
- `R = 15.25m` = 圆半径
- 噪声模型: `σ_circle = circle_tolerance` (2.0m)

### 2.2 GTSAM 自定义因子

```cpp
class CircleConstraintFactor : public gtsam::NoiseModelFactor1<gtsam::Point2> {
public:
    CircleConstraintFactor(gtsam::Key key,
                           const gtsam::Point2& center1,
                           const gtsam::Point2& center2,
                           double radius,
                           const gtsam::SharedNoiseModel& model)
        : NoiseModelFactor1(model, key),
          c1_(center1), c2_(center2), R_(radius) {}

    gtsam::Vector evaluateError(
        const gtsam::Point2& landmark,
        boost::optional<gtsam::Matrix&> H = boost::none) const override
    {
        double d1 = gtsam::distance2(landmark, c1_);
        double d2 = gtsam::distance2(landmark, c2_);

        double err1 = d1 - R_;
        double err2 = d2 - R_;

        // 选择距离更近的圆
        bool use_c1 = std::abs(err1) <= std::abs(err2);
        double err = use_c1 ? err1 : err2;
        const gtsam::Point2& center = use_c1 ? c1_ : c2_;
        double dist = use_c1 ? d1 : d2;

        if (H) {
            // Jacobian: d(err)/d(landmark)
            if (dist > 1e-6) {
                gtsam::Vector2 dir = (landmark - center) / dist;
                *H = dir.transpose();  // 1×2 Jacobian
            } else {
                *H = gtsam::Matrix::Zero(1, 2);
            }
        }

        return gtsam::Vector1(err);
    }

private:
    gtsam::Point2 c1_, c2_;
    double R_;
};
```

### 2.3 内圈/外圈区分

Skidpad 锥桶分为内圈（R - W/2）和外圈（R + W/2）：

```
内圈半径: R_inner = 15.25 - 1.5 = 13.75m
外圈半径: R_outer = 15.25 + 1.5 = 16.75m
```

可选扩展：根据路标到圆心的距离判断内/外圈，使用对应半径：

```cpp
double dist_to_center = use_c1 ? d1 : d2;
double target_R;
if (dist_to_center < R_) {
    target_R = R_ - track_width / 2.0;  // 内圈
} else {
    target_R = R_ + track_width / 2.0;  // 外圈
}
double err = dist_to_center - target_R;
```

初期实现使用单一半径 R，内/外圈区分作为后续优化。

---

## 3. 圆心位置估计

### 3.1 固定圆心 (Phase 1)

初始实现使用固定圆心位置（从配置文件读取）：

```
c1 = (0, +circle_center_dist / 2)  = (0, +9.125)
c2 = (0, -circle_center_dist / 2)  = (0, -9.125)
```

坐标系: base_link 初始系（首次 INS 有效位置为原点，初始航向为 X 轴）。

**局限**: 实际赛道可能与标准规格有偏差（±1m），固定圆心会引入系统误差。

### 3.2 可优化圆心 (Phase 2, 可选)

将圆心作为因子图中的优化变量：

```cpp
// 新增符号
constexpr unsigned char kCircleCenter = 'c';

// 圆心先验
auto center_noise = noiseModel::Diagonal::Sigmas(Vector2(2.0, 2.0));
new_factors_->add(PriorFactor<Point2>(
    Symbol('c', 0), Point2(0, +D/2), center_noise));
new_factors_->add(PriorFactor<Point2>(
    Symbol('c', 1), Point2(0, -D/2), center_noise));

// 圆心间距约束
auto dist_noise = noiseModel::Diagonal::Sigmas(Vector1(0.5));
new_factors_->add(BetweenFactor<Point2>(
    Symbol('c', 0), Symbol('c', 1),
    Point2(0, -D), dist_noise));
```

圆心位置随优化迭代收敛到实际赛道几何。

---

## 4. 交叉区域处理

### 4.1 交叉区域定义

两圆重叠区域，几何上为两圆交集：

```
交叉区域中心: (0, 0)  (两圆心连线中点)
交叉区域半径: crossover_radius ≈ sqrt(R² - (D/2)²)
            = sqrt(15.25² - 9.125²) ≈ 12.22m
```

实际有效交叉区域（车辆通过的区域）更小，约 3-5m 范围。

### 4.2 交叉区域锥桶处理

在交叉区域内，锥桶可能同时满足两个圆的约束。处理策略：

```cpp
void addCircleConstraint(int landmark_idx) {
    const auto& lm = landmarks_[landmark_idx];
    double d1 = std::hypot(lm.x - c1_.x(), lm.y - c1_.y());
    double d2 = std::hypot(lm.x - c2_.x(), lm.y - c2_.y());

    double err1 = std::abs(d1 - R_);
    double err2 = std::abs(d2 - R_);

    // 交叉区域: 两圆误差都小，使用更宽松的噪声
    if (err1 < circle_tolerance_ && err2 < circle_tolerance_) {
        // 交叉区域锥桶: 放大 σ，降低约束强度
        auto noise = noiseModel::Diagonal::Sigmas(
            Vector1(circle_tolerance_ * 2.0));
        new_factors_->add(CircleConstraintFactor(
            Symbol('l', lm.id), c1_, c2_, R_, noise));
    } else {
        // 非交叉区域: 正常约束
        auto noise = noiseModel::Diagonal::Sigmas(
            Vector1(circle_tolerance_));
        new_factors_->add(CircleConstraintFactor(
            Symbol('l', lm.id), c1_, c2_, R_, noise));
    }
}
```

### 4.3 方向感知数据关联

在交叉区域内，同一物理锥桶从不同方向接近时应正确关联。
利用车辆航向区分：

```cpp
// 在交叉区域内，额外考虑航向一致性
if (isInCrossoverZone(lm)) {
    double heading_diff = std::abs(
        normalizeAngle(vehicle_heading - lm.last_approach_heading));
    if (heading_diff > M_PI / 2) {
        // 从相反方向接近，可能是不同圈次
        // 增加关联代价，但不完全拒绝
        association_cost += crossover_heading_penalty;  // 默认 1.0
    }
}
```

---

## 5. 集成方案

### 5.1 新增方法: `addGeometryPriorFactors()`

在 `FactorGraphOptimizer` 中新增方法，按 `map_mode` 分支：

```cpp
void FactorGraphOptimizer::addGeometryPriorFactors() {
    if (cfg_.map_mode != "skidpad") return;

    // 对每个新增或更新的路标添加圆约束
    for (const auto& lm : landmarks_) {
        if (lm.obs_count < 2) continue;  // 至少观测 2 次才添加约束

        addCircleConstraint(lm);
    }
}
```

在 `TryUpdate()` 的因子添加序列中调用：

```cpp
// TryUpdate() 中:
addImuFactor();
addGnssFactor();
addSpeedFactor();
addConeFactors(current_pose);
addGeometryPriorFactors();  // 新增
runOptimization();
```

### 5.2 配置传递

`map_mode` 需要从 ROS 参数传递到 `FactorGraphConfig`：

```cpp
// location.cpp 中:
fg_config_.map_mode = params_.map_mode;  // "accel" | "skidpad" | "track"
fg_config_.circle_radius = params_.circle_radius;
fg_config_.circle_center_dist = params_.circle_center_dist;
fg_config_.circle_tolerance = params_.circle_tolerance;
```

---

## 6. 新增配置参数

| 参数 | 默认值 | 单位 | 配置键 | 说明 |
|------|--------|------|--------|------|
| 圆半径 | 15.25 | m | `map/circle_radius` | FSG 标准值 |
| 圆心距 | 18.25 | m | `map/circle_center_dist` | FSG 标准值 |
| 圆约束容差 | 2.0 | m | `map/circle_tolerance` | σ_circle |
| 交叉区域半径 | 3.0 | m | `fg/crossover_radius` | 有效交叉区域 |
| 交叉区域航向惩罚 | 1.0 | — | `fg/crossover_heading_penalty` | 方向感知代价 |
| 最小观测次数 | 2 | — | `fg/circle_min_obs` | 添加圆约束的门槛 |
| 圆心可优化 | false | — | `fg/optimize_circle_centers` | Phase 2 开关 |

注: `circle_radius`、`circle_center_dist`、`circle_tolerance` 复用现有 `location_skidpad.yaml` 中的参数。

---

## 7. 实验设计

### 7.1 消融实验

| 实验 | 圆约束 | 交叉区域处理 | 预期效果 |
|------|--------|-------------|---------|
| Baseline | 关 (仅 mapper 硬门控) | 关 | 当前性能 |
| A: 圆约束 | 开 (固定圆心) | 关 | 路标位置精度提升 |
| B: 圆约束+交叉 | 开 (固定圆心) | 开 | 交叉区域关联改善 |
| C: 可优化圆心 | 开 (优化圆心) | 开 | 适应实际赛道偏差 |

### 7.2 鲁棒性测试

| 测试 | 方法 | 指标 |
|------|------|------|
| 交叉区域遮挡 50% | 移除交叉区域 50% 锥桶 | `heading_rms_deg` 峰值 |
| 随机锥桶缺失 30% | 随机移除 30% 检测 | `position_rms_m`, `map_consistency` |
| 赛道偏差 ±1m | 修改圆心距 ±1m | `position_rms_m`（测试容差鲁棒性） |
| 4 圈完整运行 | 完整 Skidpad 赛事 | 圈间漂移、圈数计数准确性 |

### 7.3 预期结果

- 圆约束开启: 路标位置 RMS 改善 10-20%（几何先验约束漂移）
- 交叉区域处理: 交叉点航向误差峰值降低 30-50%
- 可优化圆心: 在赛道偏差 ±1m 时仍保持性能（固定圆心会退化）

---

## 8. 风险与缓解

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 实际赛道偏差大 | 圆约束引入系统误差 | 使用宽松 σ (2.0m)，Phase 2 可优化圆心 |
| 交叉区域关联错误 | 路标 ID 混乱 | 方向感知 + 放大交叉区域噪声 |
| 圆心初始化偏差 | 优化不收敛 | 使用 GNSS 初始位置校正圆心 |
| 非标准赛道 | 约束完全失效 | 检测圆拟合残差，残差过大时禁用约束 |

---

## 9. 代码变更清单

| 文件 | 变更 | 优先级 |
|------|------|--------|
| `factor_graph_types.hpp` | 新增 `CircleConstraintFactor` 类、`map_mode` 和圆参数到 `FactorGraphConfig` | P0 |
| `factor_graph_optimizer.hpp` | 新增 `addGeometryPriorFactors()` 声明、圆心成员变量 | P0 |
| `factor_graph_optimizer.cpp` | 实现 `CircleConstraintFactor`、`addGeometryPriorFactors()`、交叉区域逻辑 | P0 |
| `location.cpp` | 传递 `map_mode` 和圆参数到 `fg_config_` | P1 |
| `location_skidpad.yaml` | 参数已存在，无需修改 | — |
| `location_common.yaml` | 新增 `fg/crossover_*` 参数 | P1 |