# FSD HUAT 定位模块技术实现报告

> **版本**: v1.0  
> **涵盖范围**: `localization_core` + `localization_ros`  
> **更新时间**: 2026-04-29

---

## 1. 定位模块概述

本项目的定位模块是面向大学生方程式自动驾驶（FSD）竞赛的高精度车辆位姿估计与地图构建系统。核心任务是在赛道环境中实时输出车辆在全局地图中的二维位姿 `(x, y, yaw)`，并同步维护一个全局锥桶地图，为路径规划提供环境约束。

### 1.1 设计目标

| 目标 | 说明 |
|------|------|
| **实时性** | 主链输出 ≥ 50 Hz，ESKF 预处理 ≥ 100 Hz |
| **鲁棒性** | 支持 GNSS 信号降级、锥桶漏检、动态遮挡等异常场景 |
| **多模态适配** | 针对直线加速（Accel）、八字绕环（Skidpad）、高速循迹（Track）提供差异化参数与几何约束 |
| **可扩展性** | 双后端架构（Mapper + Factor Graph），支持影子模式对比与渐进式算法升级 |

### 1.2 包结构

```
src/localization_core/      # ROS 无关的算法核心
├── include/localization_core/
│   ├── imu_state_estimator.hpp        # 5-State EKF
│   ├── location_mapper.hpp            # KD-Tree 地图管理
│   ├── factor_graph_optimizer.hpp     # GTSAM iSAM2 SLAM
│   ├── factor_graph_types.hpp         # FG 配置与类型定义
│   ├── anomaly_state_machine.hpp      # 异常状态机
│   ├── descriptor_relocator.hpp       # 极坐标直方图重定位
│   ├── particle_relocator.hpp         # 粒子滤波重定位
│   └── types.hpp                      # 核心数据结构
├── src/
│   ├── imu_state_estimator.cpp
│   ├── location_mapper.cpp
│   ├── factor_graph_optimizer.cpp
│   ├── anomaly_state_machine.cpp
│   ├── descriptor_relocator.cpp
│   └── particle_relocator.cpp
└── test/

src/localization_ros/       # ROS 封装与运行时
├── include/localization_ros/
│   ├── location_node.hpp              # 主 ROS 节点 (~285 行头)
│   └── localization_perf_stats.hpp
├── src/
│   ├── location.cpp                   # LocationNode 实现 (~1800 行)
│   ├── main.cpp
│   └── state_estimator_node.cpp       # 独立 ESKF 节点
├── config/
│   ├── location.yaml                  # 基础配置 + 调参指南
│   ├── location_common.yaml           # 共享默认 + FG 参数
│   ├── location_track.yaml            # 赛道模式预设
│   ├── location_skidpad.yaml          # 八字模式预设
│   ├── location_accel.yaml            # 加速模式预设
│   └── state_estimator.yaml           # ESKF 参数
└── launch/
    ├── location.launch                  # 主启动（mapper/FG 可选）
    └── location_eskf.launch             # ESKF 分离架构
```

---

## 2. 整体架构设计

### 2.1 双后端架构

定位模块采用 **"Mapper 为主、Factor Graph 为辅"** 的双后端设计：

```
                    ┌─────────────────────────────────────┐
                    │         Sensors (INS/GNSS)          │
                    └──────────────┬──────────────────────┘
                                   │
              ┌────────────────────┴────────────────────┐
              │         state_estimator_node            │
              │      (5-State EKF, 100 Hz)              │
              │         ↓ localization/car_state        │
              └────────────────────┬────────────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    │      LocationNode           │
                    │   (50 Hz spin + callbacks)  │
                    └──────────────┬──────────────┘
                                   │
           ┌───────────────────────┴───────────────────────┐
           │                                               │
           ▼                                               ▼
   ┌───────────────┐                             ┌──────────────────┐
   │ LocationMapper│                             │FactorGraphOptimizer
   │ (KD-Tree Map) │                             │ (GTSAM iSAM2)    │
   │   主后端      │                             │   影子/实验后端  │
   └───────┬───────┘                             └────────┬─────────┘
           │                                               │
           │         ┌──────────────────┐                  │
           └────────►│ AnomalyStateMachine│◄─────────────────┘
                     │  (TRACKING→LOST)  │
                     └──────────────────┘
                              │
               ┌──────────────┴──────────────┐
               ▼                              ▼
    ┌─────────────────────┐      ┌─────────────────────┐
    │ DescriptorRelocator │      │ ParticleRelocator   │
    │ (极坐标直方图+RANSAC)│      │ (200粒子系统重采样)  │
    └─────────────────────┘      └─────────────────────┘
```

- **Mapper 后端（默认）**：基于 KD-Tree 的锥桶地图管理与 INS 位姿投影，计算轻量、确定性高，已作为比赛主链。
- **Factor Graph 后端（实验性）**：基于 GTSAM iSAM2 的增量式因子图优化，具备完整的协方差传播、鲁棒核函数、数据关联与重定位能力，当前以 **shadow mode** 并行运行用于数据收集与对比验证。

### 2.2 坐标系定义

| 坐标系 | 说明 | 用途 |
|--------|------|------|
| `world` | ENU 地图坐标系（以首次有效 GNSS 为原点，初始航向为 X 轴） | 全局定位与地图输出 |
| `base_link` | 车体坐标系，原点位于 IMU 安装位置 | 传感器观测输入 |
| `lidar` | 激光雷达坐标系 | 锥桶检测原始位置（通过 `lidarToIMUDist` 外参转换到 `base_link`） |

### 2.3 核心数据流

```
[INS/GNSS] ──► sensors/ins (HUAT_InsP2)
                    │
      ┌─────────────┼─────────────┐
      │             │             │
      ▼             ▼             ▼
 state_estimator  location_node   location_node
 (ESKF, 100Hz)   (imuCallback)   (INS ring buffer)
      │             │             │
      │             │             ▼
      │             │      interpolateIns() ──► 时戳对齐
      │             │             │
      └────► localization/car_state ─────────────┘
                         │
                         ▼
              ┌────────────────────┐
              │   coneCallback()   │◄── perception/lidar_cluster/detections
              └────────┬───────────┘
                       │
           ┌───────────┴───────────┐
           ▼                       ▼
    mapper_.UpdateFromCones()   feedFactorGraphCones()
           │                       │
           ▼                       ▼
    KD-Tree merge/update      FG keyframe + iSAM2
           │                       │
           └───────────┬───────────┘
                       ▼
              publishState()
                       │
        ┌──────────────┼──────────────┐
        ▼              ▼              ▼
  car_state      cone_map        TF world→base_link
```

---

## 3. EKF（扩展卡尔曼滤波）实现

### 3.1 状态向量定义

`ImuStateEstimator` 实现了一个 **5 状态 EKF**（文档中标注为 ESKF，但实际为常规 EKF，无显式误差状态拆分）：

$$
\mathbf{x} = [x,\; y,\; v_x,\; v_y,\; \theta]^T
$$

| 维度 | 含义 | 单位 |
|------|------|------|
| $x(0)$ | 全局 X 位置 | m |
| $x(1)$ | 全局 Y 位置 | m |
| $x(2)$ | 全局 X 速度 | m/s |
| $x(3)$ | 全局 Y 速度 | m/s |
| $x(4)$ | 航向角 (yaw) | rad |

### 3.2 预测步（Predict）

**输入**: `dt`, `ax`, `ay`（车体坐标系，已去除重力投影）, `gyro_z`

**运动学模型**（匀加速假设）：

```cpp
const double a_world_x = cos(yaw)*ax - sin(yaw)*ay;
const double a_world_y = sin(yaw)*ax + cos(yaw)*ay;

x(0) += x(2)*dt + 0.5*a_world_x*dt*dt;
x(1) += x(3)*dt + 0.5*a_world_y*dt*dt;
x(2) += a_world_x*dt;
x(3) += a_world_y*dt;
x(4) = NormalizeAngle(x(4) + gyro_z * gyro_scale * dt);
```

**状态转移 Jacobian** $F \in \mathbb{R}^{5 \times 5}$:

```cpp
F = I;
F(0,2) = dt;                     F(1,3) = dt;
F(0,4) = 0.5*(-sin(yaw)*ax - cos(yaw)*ay)*dt²;
F(1,4) = 0.5*( cos(yaw)*ax - sin(yaw)*ay)*dt²;
F(2,4) = (-sin(yaw)*ax - cos(yaw)*ay)*dt;
F(3,4) = ( cos(yaw)*ax - sin(yaw)*ay)*dt;
```

**过程噪声** $Q$（连续白噪声加速度模型离散化）：

```cpp
const double sa2 = accel_noise²;
const double sg2 = gyro_noise²;

Q(0,0) = Q(1,1) = sa2 * dt⁴ * 0.25;   // position
Q(2,2) = Q(3,3) = sa2 * dt²;          // velocity
Q(0,2) = Q(2,0) = sa2 * dt³ * 0.5;    // pos-vel cross
Q(1,3) = Q(3,1) = sa2 * dt³ * 0.5;
Q(4,4) = sg2 * dt²;                   // yaw
```

**协方差传播**：

$$
\mathbf{P} = F \cdot P \cdot F^T + Q
$$

传播后执行健康检查：若 `P` 出现 NaN/Inf，重置为初始方差；否则将对角线 clamp 到 `[1e-6, 1e4]`。

### 3.3 更新步（Update）

测量向量动态组装（根据 `use_gnss`, `use_velocity`, `use_yaw` 开关）：

| 传感器 | 测量 $z$ | 观测矩阵 $H$ | 噪声 $R$ |
|--------|---------|-------------|---------|
| GNSS 位置 | $[x_{meas}, y_{meas}]^T$ | $H_{0,0}=1, H_{1,1}=1$ | $\sigma_{pos}^2$ |
| INS 速度 | $[v_{x}, v_{y}]^T$ | $H_{2,2}=1, H_{3,3}=1$ | $\sigma_{vel}^2$ |
| INS 航向 | $[yaw_{meas}]$ | $H_{4,4}=1$ | $\sigma_{yaw}^2$ |

**标准卡尔曼更新方程**（Joseph 形式保证半正定性）：

```cpp
Eigen::VectorXd y = z - H * x_;           // residual
if (yaw_index >= 0) NormalizeAngle(y(yaw_index));

Eigen::MatrixXd S = H * P_ * H.transpose() + R;
Eigen::MatrixXd K = (S.ldlt().solve(H * P_)).transpose();  // LDLT 求逆保证数值稳定

x_ = x_ + K * y;
P_ = (I - K*H) * P_ * (I - K*H).transpose() + K * R * K.transpose();
```

> **注意**：航向角残差与状态更新后均执行 `NormalizeAngle` 处理，避免 $2\pi$ 跳变。

### 3.4 重力补偿与坐标转换

**重力投影去除**（FRD 车体坐标系）：

```cpp
const double g = params_.accel_gravity;
const double roll_rad  = msg.roll  * kDegToRad;
const double pitch_rad = msg.pitch * kDegToRad;

// FRD 体系下重力投影
const double ax = msg.x_acc - g * std::sin(pitch_rad);
const double ay = msg.y_acc - (-g * std::sin(roll_rad) * std::cos(pitch_rad));
```

**WGS84 → ENU → MapFrame**：

1. 以第一条有效 INS 消息的 `(lat, lon, alt)` 为原点，通过完整的椭球模型（WGS84）转换到 ENU。
2. 以初始航向 `azimuth` 为基准，将 ENU 旋转到地图对齐坐标系：
   ```cpp
   origin_rot_rad_ = (standard_azimuth_deg_ - 90.0) * kDegToRad;
   x_map = east * cos(origin_rot) - north * sin(origin_rot);
   y_map = east * sin(origin_rot) + north * cos(origin_rot);
   ```

### 3.5 FSSIM 风格低速运动学修正

低速时（$< 1.5 \text{ m/s}$），动力学模型因滑移角计算中的除零和小速度不稳定，引入 **自行车模型混合修正**：

```cpp
// 混合因子: 0 = 纯运动学, 1 = 纯动力学
const double blend = clamp((speed - 1.5) / 1.0, 0.0, 1.0);

// 运动学模型横向速度
vy_kin = tan(delta) * |vx| * l_r / L;

// 混合修正
vy_corrected = blend * vy_body + (1.0 - blend) * vy_kin;
```

| 参数 | 值 | 含义 |
|------|-----|------|
| `wheelbase` | 1.55 m | 轴距 |
| `cg_to_rear` | 0.78 m | 重心到后轴距离 |
| `blend_speed` | 1.5 m/s | 纯运动学阈值 |
| `blend_range` | 1.0 m/s | 混合过渡带宽 |

### 3.6 EKF 关键参数

```yaml
# 过程噪声
accel_noise: 1.0          # m/s²
 gyro_noise: 0.05         # rad/s

# 观测噪声
meas_pos_noise: 0.4       # m
meas_vel_noise: 0.2       # m/s
meas_yaw_noise: 0.05      # rad

# 初始方差
init_pos_var: 1.0
init_vel_var: 1.0
init_yaw_var: 0.1

# 时序保护
min_dt: 0.001
max_dt: 0.1
```

---

## 4. SLAM / 因子图优化实现

### 4.1 技术选型：GTSAM iSAM2

因子图后端基于 **GTSAM iSAM2**（Incremental Smoothing and Mapping 2），选择理由：

- **增量式优化**：仅对新加入的因子和变量做局部更新，避免全局批量优化的高延迟。
- **自然的不确定性传播**：变量节点自动维护边缘协方差，无需手动维护 EKF 的协方差矩阵。
- **灵活的因子类型**：支持自定义因子（如八字绕环的 `CircleConstraintFactor`）。

### 4.2 图结构：节点与边

#### 变量节点（GTSAM Symbols）

| 符号 | GTSAM 类型 | 含义 |
|------|-----------|------|
| $x_k$ (`'x'`) | `gtsam::Pose2` | 第 $k$ 个关键帧的车辆位姿 |
| $v_k$ (`'v'`) | `gtsam::Vector2` | 第 $k$ 个关键帧的车辆速度 |
| $l_i$ (`'l'`) | `gtsam::Point2` | 第 $i$ 个锥桶路标 |

#### 因子（边）定义

| 因子类型 | 连接变量 | 噪声模型 | 鲁棒核 |
|---------|---------|---------|--------|
| `PriorFactor<Pose2>` | $x_0$ | $\sigma = [0.1, 0.1, 0.05]$ | — |
| `PriorFactor<Vector2>` | $v_0$ | $\sigma = [1.0, 1.0]$ | — |
| `BetweenFactor<Pose2>` | $x_{k-1} \to x_k$ | $\sigma_{xy}\sqrt{dt}, \sigma_{\theta}\sqrt{dt}$ | — |
| `BetweenFactor<Vector2>` | $v_{k-1} \to v_k$ | $\sigma_{vel}\sqrt{dt}$ | — |
| `PriorFactor<Pose2>` (GNSS) | $x_k$ | 质量自适应 $\sigma$ | **Cauchy** (`cauchy_gnss = 5.0`) |
| `PriorFactor<Vector2>` (Speed) | $v_k$ | $\sigma_{speed}$ | **Huber** (`huber_speed = 1.345`) |
| `BearingRangeFactor<Pose2,Point2>` | $x_k \to l_i$ | 距离相关噪声 | **Huber** (`huber_cone = 2.0`) |
| `CircleConstraintFactor` | $l_i$ | $\sigma_{circle}$ | — |

### 4.3 关键帧触发策略

```cpp
bool shouldCreateKeyframe(const Pose2& pose, double timestamp) const {
  const double dist  = hypot(pose.x - last.x, pose.y - last.y);
  const double dyaw  = abs(atan2(sin(pose.theta - last.theta), cos(...)));
  const double dt    = timestamp - last_keyframe_time_;

  return (dist >= 1.0) || (dyaw >= 0.1) || (dt >= 0.5);
}
```

满足 **距离 ≥ 1.0 m**、**航向变化 ≥ 0.1 rad**、**时间 ≥ 0.5 s** 任一条件即生成新关键帧。

### 4.4 IMU 预积分因子

不同于高维 IMU 预积分（如 15-state IMU 模型），本项目使用 **2D 速度-航向角速率死 reckoning**：

```cpp
// 在 AddImuMeasurement() 中累积车体坐标系下的位移增量
const double mid_theta = accum_dtheta_ + wz * dt * 0.5;
accum_dx_ += cos(mid_theta) * v_forward * dt;
accum_dy_ += sin(mid_theta) * v_forward * dt;
accum_dtheta_ += wz * dt;

// 在关键帧时刻构造 BetweenFactor<Pose2>
gtsam::Pose2 delta(accum_dx_, accum_dy_, accum_dtheta_);
auto noise = Diagonal::Sigmas(Vector3(sigma_imu_xy * sqrt(dt), ...));
new_factors_->add(BetweenFactor<Pose2>(x_{k-1}, x_k, delta, noise));
```

> 该简化模型假设地面平坦、无侧滑，适合 FSD 低速赛道场景。

### 4.5 GNSS 质量自适应先验

GNSS 质量分为三级，动态调整噪声：

```cpp
enum class GnssQuality { GOOD, MEDIUM, POOR, INVALID };

// 分级标准
GOOD   : status ≥ 2, nsv ≥ 12, age < 5   → σ = 0.5 m
MEDIUM : status ≥ 2, nsv ≥ 8,  age < 15  → σ = 2.0 m
POOR   : otherwise valid                → σ = 10.0 m
```

同时应用 **Cauchy 鲁棒核**，降低 GNSS 多路径或 RTK 失锁时的异常值影响。

### 4.6 锥桶观测因子与数据关联

#### 4.6.1 观测预处理

锥桶检测从 `base_link` 转换到极坐标 `(range, bearing)`：

```cpp
const double lx_body = obs.range * cos(obs.bearing);
const double ly_body = obs.range * sin(obs.bearing);
const double gx = pose.x + cos(pose.theta)*lx_body - sin(pose.theta)*ly_body;
const double gy = pose.y + sin(pose.theta)*lx_body + cos(pose.theta)*ly_body;
```

#### 4.6.2 多维代价数据关联

```cpp
double cost = w_maha * d²                        // 马氏距离（欧氏距离平方）
          + w_color * (1 - confusion[obs_c][lm_c]) // 颜色混淆代价
          + w_topo  * topo_penalty;                // 拓扑侧别惩罚
```

当前纯 LiDAR 模式下 `w_color = 0.0, w_topo = 0.0`，退化为纯几何最近邻。

#### 4.6.3 距离相关噪声模型

锥桶测距噪声随距离增大：

$$
\sigma_r = \sigma_{range\_base} + \sigma_{range\_scale} \cdot r = 0.1 + 0.02 \cdot r \;\text{[m]}
$$

### 4.7 八字绕环几何约束（自定义因子）

针对 Skidpad 模式，定义了 `CircleConstraintFactor` —— 将锥桶路标约束到两个已知圆之一：

```cpp
// 自定义一元因子：误差 = min(|dist(p,c1)-R|, |dist(p,c2)-R|)
class CircleConstraintFactor : public gtsam::NoiseModelFactor1<gtsam::Point2> {
  gtsam::Point2 c1_, c2_;
  double radius_;

  gtsam::Vector evaluateError(const gtsam::Point2& p,
                              boost::optional<gtsam::Matrix&> H = boost::none) const override {
    double d1 = (p - c1_).norm();
    double d2 = (p - c2_).norm();
    double e1 = fabs(d1 - radius_);
    double e2 = fabs(d2 - radius_);
    // 选择更近的圆，并计算解析 Jacobian
    ...
  }
};
```

| 参数 | 值 | 含义 |
|------|-----|------|
| `circle_radius` | 15.25 m | 圆弧半径 |
| `circle_center_dist` | 18.25 m | 两圆心间距 |
| `circle_sigma` | 1.0 m | 圆约束噪声 |

---

## 5. 地图管理（KD-Tree Mapper）

### 5.1 地图表示

`LocationMapper` 使用 **PCL `KdTreeFLANN`** 维护全局锥桶地图：

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;   // 空间索引
std::vector<int>              point_ids_;         // 锥桶 ID
std::vector<int>              point_obs_counts_;  // 观测次数（加权合并用）
std::vector<std::uint8_t>     point_types_;       // 颜色类型
std::vector<double>           point_confidences_; // 置信度
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
```

### 5.2 更新流程

```
UpdateFromCones()
    ├── 1. INS 质量门控检查（status ≥ 2, nsv ≥ 8, age < 30）
    ├── 2. 每个检测锥桶:
    │      ├── 几何过滤（bbox 尺寸、Y轴偏移、圆验证）
    │      ├── 置信度过滤（min_confidence_to_add / min_confidence_to_merge）
    │      ├── KD-Tree 最近邻搜索（merge_distance 半径）
    │      ├── 若找到匹配: 加权平均更新位置 (obs_count 权重)
    │      └── 若无匹配且未冻结: 插入新锥桶
    ├── 3. 地图维护:
    │      ├── 超限清理（max_map_size，优先删除低 obs_count）
    │      ├── 缺锥插值（missing_cone_fallback）
    │      └── 短路径抑制（short_path_suppression）
    └── 4. 输出局部地图（local_cone_range 范围内）
```

### 5.3 几何鲁棒性增强

针对纯 LiDAR 模式（`vision_mode.enabled = false`）的几何增强策略：

| 功能 | 说明 |
|------|------|
| **堆叠锥去重** | Z 轴高度差 + XY 栅格（0.5 m cell）去重，置信度衰减 0.8 |
| **缺锥插值** | 沿赛道方向预测缺失锥桶，最大插值距离 8 m，最多连续补 3 个 |
| **短路径抑制** | 拒绝长度 < 3 m 且锥桶数 < 3 的虚假路径段 |
| **回环稳定性** | 最小回环长度 15 m，几何一致性验证（间距、赛道宽度） |

### 5.4 地图漂移检测与回退

```yaml
map_drift:
  enabled: true
  min_match_ratio: 0.15
  max_mean_match_distance: 2.0
  bad_frames_to_trigger: 15
  action: "rollback"   # 可选 rollback | reset | freeze
```

当连续多帧锥桶匹配率过低或平均匹配距离过大时，触发 **checkpoint rollback**，将地图恢复到上一稳定状态。

---

## 6. 异常检测与重定位

### 6.1 五层异常状态机

```
TRACKING ──[chi² > 15 或 match_ratio < 0.3 持续2s]──► DEGRADED
    ▲                                                    │
    │                                                    ▼
    └─────────────────────────────────────────────── LOST
    │         (3s 超时)                      (5s 超时)
    │              ▼                               ▼
    │           RELOC_A  ──[失败]─────────────► RELOC_B
    │      (描述子重定位)                        (粒子滤波)
    │              │                               │
    └──────────────┴──[成功]───────────────────────┘
```

| 状态 | 行为 | 恢复条件 |
|------|------|---------|
| **TRACKING** | 正常建图与定位，构建描述子数据库 | — |
| **DEGRADED** | 增大协方差因子，保守更新 | chi² < 5 且匹配恢复 |
| **LOST** | 停止建图，进入重定位 | 重定位成功 |
| **RELOC_A** | 极坐标直方图描述子 + RANSAC SE(2) | 内点数 ≥ 4 |
| **RELOC_B** | 200 粒子系统重采样 + 扩散 | 加权 std < 0.5 m / 0.1 rad |

### 6.2 描述子重定位（DescriptorRelocator）

**极坐标直方图描述子**：

```
 rings (5)  ×  sectors (12)  ×  channels (3)
  - Channel 0: BLUE
  - Channel 1: RED / YELLOW
  - Channel 2: other / none
```

描述子经 L2 归一化后，通过 **余弦距离** 在数据库（最多 200 条）中检索 Top-5 候选。对每个候选执行 **2 点 RANSAC**（最多 100 次迭代）：

1. 采样 2 个当前局部路标 → 找到数据库中最近邻路标
2. 估计 SE(2) 变换（平移 + 旋转）
3. 尺度一致性检查（尺度差异 > 30% 拒绝）
4. 内点计数（阈值 1.5 m），保留最佳候选

### 6.3 粒子滤波重定位（ParticleRelocator）

| 参数 | 值 |
|------|-----|
| 粒子数 | 200 |
| 初始化散布 | $\sigma_{xy}=3.0$ m, $\sigma_{\theta}=0.3$ rad |
| 观测似然 | 高斯，以最近路标距离为误差，$\sigma=2.0$ m |
| 重采样 | 系统重采样 + 扩散噪声 ($\sigma_{xy}=0.1$ m, $\sigma_{\theta}=0.02$ rad) |
| 收敛判定 | 加权位置 std < 0.5 m 且航向 std < 0.1 rad |

---

## 7. 传感器数据流与同步

### 7.1 传感器输入

| 传感器 | 消息类型 | 来源 | 频率 |
|--------|---------|------|------|
| INS/GNSS (INS5711DAA) | `autodrive_msgs/HUAT_InsP2` | `sensors/ins` | ~100 Hz |
| LiDAR 锥桶检测 | `autodrive_msgs/HUAT_ConeDetections` | `perception/lidar_cluster/detections` | ~10 Hz |
| 外部车辆状态（可选） | `autodrive_msgs/HUAT_CarState` | `localization/car_state` | ~100 Hz |

### 7.2 INS 质量门控

```cpp
if (status < min_ins_status)      return DROP;  // status < 2
if (nsv < min_satellite_count)    return DROP;  // nsv < 8
if (age > max_diff_age)           return DROP;  // age > 30 (3.0 s)
```

### 7.3 时戳对齐与插值

由于锥桶检测（~10 Hz）与 INS（~100 Hz）频率不同，使用 **200 帧 INS 环形缓冲区**（约 2 秒）进行线性插值：

```cpp
struct StampedIns {
  ros::Time stamp;
  localization_core::Asensing data;
};
std::deque<StampedIns> ins_buffer_;   // size = 200

// 在 coneCallback() 中插值到检测消息的时间戳
interpolateIns(cone_stamp, out_ins);
```

插值规则：连续量（位置、速度、角度）线性插值；离散量（status, nsv, age）取最近邻。

### 7.4 TF 发布与时序监控

发布 `world → base_link` 的 TF，并监控以下指标：

| 指标 | 阈值 | 异常处理 |
|------|------|---------|
| 发布延迟 (`now - stamp`) | 0.10 s | 警告 + 计数 |
| 未来时间戳 (`stamp > now`) | 0.02 s | 警告 + 计数 |
| 相邻发布间隔 | 0.20 s | 警告 + 计数 |
| INS 时戳回退 | 5 ms | 拒绝消息 + 计数 |

---

## 8. 关键参数配置汇总

### 8.1 ESKF 参数 (`state_estimator.yaml`)

```yaml
accel_noise: 1.0              # m/s²
 gyro_noise: 0.05             # rad/s
meas_pos_noise: 0.4           # m
meas_vel_noise: 0.2           # m/s
meas_yaw_noise: 0.05          # rad

kinematic_correction:
  enable: true
  wheelbase: 1.55
  cg_to_rear: 0.78
  blend_speed: 1.5
  blend_range: 1.0
```

### 8.2 Mapper 参数 (`location.yaml`)

```yaml
map:
  mode: track
  merge_distance: 2.5         # m
  max_map_size: 500
  min_obs_to_keep: 2
  local_cone_range: 50.0
  min_confidence_to_add: 0.35
  min_confidence_to_merge: 0.15
```

### 8.3 Factor Graph 参数 (`location_common.yaml`)

```yaml
fg:
  shadow_mode: true           # true: mapper 主输出，FG 影子运行
  keyframe_dist: 1.0
  keyframe_yaw: 0.1
  keyframe_dt: 0.5

  sigma_imu_xy: 0.05
  sigma_imu_theta: 0.01
  sigma_gnss_good: 0.5
  sigma_gnss_medium: 2.0
  sigma_gnss_poor: 10.0

  huber_cone: 2.0
  cauchy_gnss: 5.0

  w_maha: 1.0
  w_color: 0.0                # 纯 LiDAR 模式禁用
  w_topo: 0.0
  gate_threshold: 15.0
```

---

## 9. 技术挑战与解决方案

### 9.1 低速动力学不稳定

**挑战**：车速低于 2 m/s 时，IMU 积分受轮胎滑移、零偏漂移影响，横向速度估计发散。  
**方案**：引入 FSSIM 风格运动学修正，在 0–1.5 m/s 纯运动学、1.5–2.5 m/s 线性混合、> 2.5 m/s 纯动力学，避免低速除零与滑移角不稳定。

### 9.2 GNSS 多路径与 RTK 失锁

**挑战**：赛道环境中建筑物、树木遮挡导致 GNSS 出现跳变或降级。  
**方案**：
- 三级质量门控（GOOD/MEDIUM/POOR），动态调整 GNSS 先验噪声 0.5/2.0/10.0 m。
- Cauchy 鲁棒核削弱异常 GNSS 观测对图优化的拉扯。
- Mapper 运行态保护：连续失败时降级为 INS_ONLY，避免坏观测污染地图。

### 9.3 锥桶数据关联歧义

**挑战**：相邻锥桶间距小（~5 m），纯几何最近邻易误匹配。  
**方案**：
- 多维代价函数预留颜色-拓扑关联接口（`w_color`, `w_topo`），待视觉分类启用后可无缝接入。
- 马氏距离门控 + 快速拒绝（`merge_distance` 预筛选）。

### 9.4 地图漂移与累积误差

**挑战**：长赛道（> 200 m）纯 INS+LiDAR 建图存在航向漂移，导致回环时地图错位。  
**方案**：
- 地图冻结策略（第一圈建图后冻结新锥桶插入）。
- 漂移检测（匹配率 + 平均距离监控）触发 checkpoint rollback。
- Factor Graph 后端提供未来迁移路径：iSAM2 增量优化天然具备协方差传播与回环修正能力。

### 9.5 八字绕环的几何约束

**挑战**：Skidpad 模式锥桶严格分布在两个圆上，纯观测融合无法利用该强先验。  
**方案**：自定义 `CircleConstraintFactor`（GTSAM 一元因子），将路标约束到最近圆上，显著降低圆心附近路标的不确定性。

### 9.6 重定位实时性

**挑战**：车辆被遮挡或漂移后，需要在有限时间内恢复定位。  
**方案**：
- 两级重定位：描述子重定位（RELOC_A，3 s 超时）→ 粒子滤波（RELOC_B，5 s 超时）。
- 描述子数据库滑动窗口（200 条），保证检索速度。
- 粒子滤波系统重采样避免粒子退化。

---

## 10. 代码结构与关键实现片段

### 10.1 EKF 核心循环 (`imu_state_estimator.cpp`)

```cpp
bool ImuStateEstimator::Process(const Asensing& msg, double stamp_sec, CarState* out) {
  if (!initialized_) { Initialize(msg, stamp_sec); return true; }

  double dt = clamp(stamp_sec - last_time_sec_, min_dt, max_dt);

  // 1. 重力补偿
  const double ax = msg.x_acc - g*sin(pitch);
  const double ay = msg.y_acc - (-g*sin(roll)*cos(pitch));

  // 2. 预测
  Predict(dt, ax, ay, msg.z_angular_velocity);

  // 3. 低速运动学修正
  if (enable_kinematic_correction) {
    ApplyKinematicCorrection(steering);
  }

  // 4. 组装观测并更新
  Eigen::VectorXd z(meas_dim);
  Eigen::MatrixXd H = MatrixXd::Zero(meas_dim, 5);
  Eigen::MatrixXd R = MatrixXd::Zero(meas_dim, meas_dim);
  // ... 填充 GNSS / 速度 / 航向观测 ...
  Update(z, H, R, yaw_index);

  // 5. 输出 CarState
  out->car_state.x = x_(0);
  out->car_state.y = x_(1);
  out->car_state.theta = NormalizeAngle(x_(4));
  out->Vy = -x_(2)*sin(yaw) + x_(3)*cos(yaw);  // 车体横向速度
  return true;
}
```

### 10.2 因子图更新循环 (`factor_graph_optimizer.cpp`)

```cpp
bool FactorGraphOptimizer::TryUpdate(const Pose2& current_pose, double timestamp) {
  if (!initialized_) { initializeFirstKeyframe(current_pose, timestamp); return true; }
  if (!shouldCreateKeyframe(current_pose, timestamp)) return false;

  keyframe_idx_++;

  // 添加各类因子
  addImuFactor();       // BetweenFactor<Pose2> + 速度连续性
  addGnssFactor();      // PriorFactor<Pose2> with Cauchy
  addSpeedFactor();     // PriorFactor<Vector2> with Huber
  addConeFactors(current_pose);  // BearingRangeFactor for each cone
  addGeometryPriorFactors();

  // 插入初值并优化
  new_values_->insert(Symbol('x', keyframe_idx_), Pose2(current_pose));
  new_values_->insert(Symbol('v', keyframe_idx_), Vector2(last_speed_, 0));
  runOptimization();    // isam_->update() + calculateEstimate()

  // 健康评估与异常处理
  last_chi2_normalized_ = computeChi2Normalized();
  anomaly_sm_.Evaluate(last_chi2_normalized_, match_ratio, gnss_quality_, timestamp);

  // 重定位执行（RELOC_A / RELOC_B）
  if (anomaly_sm_.GetState() == AnomalyState::RELOC_A) { ... }
  if (anomaly_sm_.GetState() == AnomalyState::RELOC_B) { ... }

  return true;
}
```

### 10.3 数据关联与路标管理 (`factor_graph_optimizer.cpp`)

```cpp
int FactorGraphOptimizer::findOrCreateLandmark(const ConeObservation& obs,
                                               const Pose2& pose) {
  // 观测转换到全局坐标
  const double gx = pose.x + cos(pose.theta)*lx_body - sin(pose.theta)*ly_body;
  const double gy = pose.y + sin(pose.theta)*lx_body + cos(pose.theta)*ly_body;

  // 多维代价搜索
  double best_cost = cfg_.gate_threshold;
  int best_idx = -1;
  for (size_t i = 0; i < landmarks_.size(); ++i) {
    const double d2 = squaredDistance(landmarks_[i], gx, gy);
    if (d2 > cfg_.merge_distance * cfg_.merge_distance) continue;

    double cost = cfg_.w_maha * d2;
    cost += cfg_.w_color * (1.0 - color_confusion[obs_c][lm_c]);
    if (oppositeSide(obs_c, lm_c)) cost += cfg_.w_topo * cfg_.topo_penalty;

    if (cost < best_cost) { best_cost = cost; best_idx = i; }
  }

  if (best_idx >= 0) {
    landmarks_[best_idx].obs_count++;
    return best_idx;   // MATCH
  }

  // CREATE NEW
  if (landmarks_.size() >= cfg_.max_landmarks) return -1;
  landmarks_.push_back({next_id_++, gx, gy, obs.color_type, 1});

  Symbol lm_key('l', idx);
  new_values_->insert(lm_key, Point2(gx, gy));
  new_factors_->addPrior(lm_key, Point2(gx, gy),
                         Diagonal::Sigmas(Vector2(landmark_init_sigma, ...)));
  return idx;
}
```

### 10.4 ROS 节点数据流 (`location_node.hpp` 架构)

```cpp
class LocationNode {
  // 双后端
  localization_core::LocationMapper mapper_;
  std::unique_ptr<localization_core::FactorGraphOptimizer> fg_optimizer_;

  // 传感器回调
  void imuCallback(const autodrive_msgs::HUAT_InsP2::ConstPtr& msg);
  void carstateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr& msg);
  void coneCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr& msg);

  // INS 时戳对齐
  std::deque<StampedIns> ins_buffer_;
  bool interpolateIns(const ros::Time& target, Asensing& out) const;

  // 运行态保护
  MapperRuntimeState mapper_state_;
  void updateMapperStateMachine(bool frame_good);

  // 漂移监控
  bool evaluateDriftAndRecover(const MapUpdateStats& stats, const ros::Time& stamp);
};
```

---

## 11. 测试与验证

| 测试类型 | 命令 | 说明 |
|---------|------|------|
| 单元测试 | `catkin run_tests localization_core` | EKF 初始化、FG 组件基础测试 |
| 性能测试 | `catkin run_tests --cmake-args -DCMAKE_BUILD_TYPE=Release` | iSAM2 优化耗时、chi² 监控 |
| 离线回放 | `roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to.bag` | 全链路回放验证 |
| 调参观测 | `rostopic echo /localization/car_state \| grep -E "(x\|y\|yaw)"` | 实时位姿监控 |
| 地图可视化 | `rviz -d $(rospack find fsd_visualization)/rviz/world_planning.rviz` | 锥桶地图与定位可视化 |

---

## 12. 总结与演进路线

当前定位模块已形成 **"ESKF 高频平滑 + KD-Tree Mapper 主链生产 + Factor Graph 影子演进"** 的三层架构：

1. **ESKF** 提供 100 Hz 的高频状态估计，融合 INS/GNSS，具备低速运动学修正与完整的数值鲁棒性保障。
2. **KD-Tree Mapper** 作为成熟主链，通过几何过滤、地图冻结、漂移回退等机制，在竞赛场景中表现出高可靠性。
3. **Factor Graph (GTSAM iSAM2)** 作为下一代后端储备，已完成 IMU 预积分、GNSS 鲁棒先验、锥桶 Bearing-Range 因子、颜色-拓扑软关联、Skidpad 圆约束、两级重定位等完整能力链，待充分验证后可从 shadow mode 切换为主链。

未来演进方向包括：视觉颜色分类启用后激活 `w_color` 与 `w_topo` 权重、多假设数据关联、显式回环闭合因子（非连续关键帧之间的 `BetweenFactor`）、以及基于 FG 的在线外参标定。
