# LiDAR 感知处理流程

> 基于 `perception_core/src/utility.cpp` 的 `Process()` 函数和 `lidar_cluster_example.yaml` 配置。
> 最后更新：2026-02-09

## 流程总览

```
原始点云
  │
  ▼
┌─────────────────────────────┐
│ Phase 1: ROI 裁剪 + 预处理   │
│  ├─ CropBox ROI 裁剪         │  ✅ 始终开启
│  └─ 强度滤波                 │  ❌ 关闭
└─────────────────────────────┘
  │
  ▼
┌─────────────────────────────┐
│ Phase 2: 地面分割             │
│  ├─ FGS (快速地面分割)        │  ✅ 当前选用
│  ├─ RANSAC                   │  ❌ 未选用
│  └─ Patchwork++              │  ❌ 未选用
└─────────────────────────────┘
  │
  ▼ (非地面点)
┌─────────────────────────────┐
│ Phase 3: 后处理滤波           │
│  ├─ 柱状障碍物滤波 (z跨度)    │  ✅ 开启
│  ├─ 距离自适应体素降采样      │  ✅ 开启
│  └─ SOR 统计离群点剔除       │  ❌ 关闭
└─────────────────────────────┘
  │
  ▼
┌─────────────────────────────┐
│ Phase 4: 多帧累积             │  ❌ 关闭
└─────────────────────────────┘
  │
  ▼
┌─────────────────────────────┐
│ Phase 5: 聚类                 │
│  ├─ 欧式聚类 (FEC加速)       │  ✅ 当前选用
│  ├─ DBSCAN                   │  ❌ 未选用
│  └─ 自适应聚类大小           │  ✅ 开启
└─────────────────────────────┘
  │
  ▼ (候选簇)
┌─────────────────────────────┐
│ Phase 6: 置信度评分           │
│  ├─ 特征提取 (几何/PCA/强度)  │  ✅ 始终开启
│  ├─ 多维置信度评分           │  ✅ 始终开启
│  ├─ 几何模型拟合加分         │  ✅ 开启
│  ├─ 距离自适应Y轴ROI剔除     │  ✅ 开启
│  └─ 赛道语义评分 (邻域上下文) │  ✅ 开启
└─────────────────────────────┘
  │
  ▼ (锥桶检测)
┌─────────────────────────────┐
│ Phase 7: 后检测精炼           │
│  ├─ 近距离去重 (NMS)         │  ✅ 开启
│  ├─ 时序跟踪器 (Hungarian)   │  ✅ 开启
│  └─ 拓扑修复 (插值+离群剔除) │  ✅ 开启
└─────────────────────────────┘
  │
  ▼
最终输出 → /detections
```

---

## 各阶段详细说明

### Phase 1: ROI 裁剪 + 预处理

| 步骤 | 开关 | 当前状态 | 说明 |
|------|------|---------|------|
| CropBox ROI 裁剪 | `filters.use_cropbox` | ✅ 开启 | 按 `roi.mode` 选择 X/Y/Z 范围，单次遍历裁剪 |
| 强度滤波 | `filters.intensity.enable` | ❌ 关闭 | 去除低反射率噪声点 |

**ROI 范围（track 模式）：**

```yaml
roi.mode: track
  x: [0.1, 50]    # 前向 0.1~50m
  y: [-10, 10]     # 侧向 ±10m
  z: [-0.5, 0.7]   # 高度
```

---

### Phase 2: 地面分割

| 方法 | 开关 | 当前状态 | 特点 |
|------|------|---------|------|
| **FGS** | `ground_method: fgs` | ✅ 选用 | 极坐标栅格，O(n)，<5ms/帧 |
| RANSAC | `ground_method: ransac` | ❌ 未选用 | 分区+自适应阈值，稳定但较慢 |
| Patchwork++ | `ground_method: patchworkpp` | ❌ 未选用 | 最精细，耗时最高 |

**FGS 关键参数：**

```yaml
fgs:
  num_sectors: 32              # 扇区数
  num_bins: 80                 # 每扇区 bin 数
  th_ground: 0.08              # 地面距离阈值 (m)
  th_ground_far: 0.15          # 远距离阈值 (m)
  enable_temporal_smoothing: true  # 帧间 EMA 平滑（抑制闪烁）
  temporal_alpha: 0.5          # EMA 系数
  enable_adaptive_alpha: true  # S弯自适应
```

**看门狗：** `ground_watchdog.enable: true`，单帧超 8ms 连续 5 帧告警。

---

### Phase 3: 后处理滤波（地面分割后）

| 步骤 | 开关 | 当前状态 | 说明 |
|------|------|---------|------|
| **柱状障碍物滤波** | `filters.obstacle_height.enable` | ✅ 开启 | 2D 栅格统计 z 跨度，去除树/墙 |
| **距离自适应体素** | `filters.distance_adaptive_voxel.enable` | ✅ 开启 | 近处大体素降采样，远处小体素保精度 |
| 普通体素 | `filters.voxel.enable` | ❌ 关闭 | 被距离自适应体素替代 |
| 自适应体素 | `filters.adaptive_voxel.enable` | ❌ 关闭 | 有性能问题，已弃用 |
| SOR | `filters.sor.enable` | ❌ 关闭 | 地面分割已足够好 |

**柱状障碍物滤波参数：**

```yaml
obstacle_height:
  grid_size: 0.5          # 栅格大小 (m)
  max_z_span: 0.4         # 最大 z 跨度 (m)，锥桶~0.325m
  min_points_to_judge: 3  # 最少点数才判定
  min_distance: 10.0      # 仅 >10m 生效
```

**距离自适应体素参数：**

```yaml
distance_adaptive_voxel:
  near_leaf: 0.08         # <10m 体素 8cm
  far_leaf: 0.03          # >10m 体素 3cm
  dist_threshold: 10.0    # 分界距离
```

---

### Phase 4: 多帧累积

| 步骤 | 开关 | 当前状态 | 说明 |
|------|------|---------|------|
| 远处点多帧累积 | `cluster.multi_frame.enable` | ❌ 关闭 | 车动/掉帧时易堆叠成假簇 |

---

### Phase 5: 聚类

| 方法 | 开关 | 当前状态 | 说明 |
|------|------|---------|------|
| **欧式聚类** | `cluster.method: euclidean` | ✅ 选用 | 分段距离阈值 + FEC 加速 |
| DBSCAN | `cluster.method: dbscan` | ❌ 未选用 | 密度聚类，可选 |

**欧式聚类参数：**

```yaml
cluster:
  # 距离分段 → 聚类容差（由近到远）
  distance_segments: [5, 10, 15, 25, 35, 50]
  cluster_tolerance: [0.15, 0.3, 0.5, 0.5, 0.4, 0.3, 0.25]

  # 自适应聚类大小
  adaptive_size:
    enable: true
    near_min_size: 3    near_max_size: 100   # <5m
    far_min_size: 2     far_max_size: 30     # >50m

  # FEC 快速聚类
  fec:
    enable: true
    quality: 0.3        # 0=最快, 0.9=最精确
```

---

### Phase 6: 置信度评分

这是最复杂的阶段，分多个子步骤：

#### 6a. 特征提取

对每个聚类计算：
- **几何特征**：长/宽/高、面积、体积、纵横比
- **PCA 形状特征**：elongation、planarity、verticality、**linearity**
- **强度特征**：均值、标准差、最大值
- **位置特征**：质心距离、地面高度

#### 6b. 多维置信度评分

| 维度 | 权重 | 关键参数 |
|------|------|---------|
| 尺寸 | 0.35 | `min_height: 0.05`, `max_height: 0.75`, `max_area: 0.25` |
| 形状 | 0.30 | `min_aspect_ratio: 0.9`, `min_verticality: 0.7`, `max_linearity: 0.85` |
| 密度 | 0.20 | `min_density_near: 50`, `min_density_far: 10` |
| 强度 | 0.05 | `min_intensity_mean: 15.0` |
| 位置 | 0.10 | `max_box_altitude: 0.05` |

#### 6c. 几何模型拟合

| 开关 | 当前状态 | 说明 |
|------|---------|------|
| `confidence.enable_model_fitting` | ✅ 开启 | 拟合成功 +0.2，失败 -0.05 |

仅对 ≥8 点的聚类执行。

#### 6d. 距离自适应 Y 轴 ROI

| 开关 | 当前状态 | 说明 |
|------|---------|------|
| `roi.adaptive_y.enable` | ✅ 开启 | 远处收窄 Y 范围，减少赛道外假锥桶 |

```
X ≤ 15m  → Y 限制 ±10m（近场宽）
X ≥ 50m  → Y 限制 ±4m （远场窄）
中间     → 线性插值
```

#### 6e. 距离自适应置信度门槛

```yaml
min_confidence_near: 0.2     # <8m 门槛
min_confidence_far: 0.45     # >25m 门槛
confidence_ramp_start: 8.0   # 开始升高
confidence_ramp_end: 25.0    # 达到最高
```

远处要求更高置信度才判定为锥桶。

#### 6f. 赛道语义评分（邻域上下文）

| 开关 | 当前状态 | 说明 |
|------|---------|------|
| `confidence.track_semantic.enable` | ✅ 开启 | 权重 15%，利用赛道几何先验 |

评分维度：
- **间距评分**：最近邻距离是否接近预期锥桶间距 (5m)
- **宽度评分**：对侧最近邻距离是否接近赛道宽度 (3m)
- **孤立惩罚**：8m 内无邻居 → 低分

**硬剔除规则：**
- 8m 内邻居数 = 0 → `confidence = 0`（直接剔除）
- 最近邻距离 > 12m → `confidence = 0`（极度孤立）

---

### Phase 7: 后检测精炼

| 步骤 | 开关 | 当前状态 | 说明 |
|------|------|---------|------|
| **近距离去重** | `dedup.enable` | ✅ 开启 | NMS，半径 0.5m，按置信度贪心抑制 |
| **时序跟踪器** | `tracker.enable` | ✅ 开启 | Hungarian 关联，连续 3 帧确认 |
| **拓扑修复** | `topology.enable` | ✅ 开启 | PCA 方向估计，插值填补缺口 |

**跟踪器参数：**

```yaml
tracker:
  association_threshold: 2.0   # 关联距离 (m)，需容忍帧间位移
  confirm_frames: 3            # 连续 3 帧确认
  delete_frames: 5             # 连续 5 帧丢失则删除
  only_output_confirmed: true  # 仅输出已确认锥桶
  confirmed_confidence_boost: 0.1
```

**拓扑修复参数：**

```yaml
topology:
  max_same_side_spacing: 5.0     # 同侧最大间距 (m)
  min_track_width: 2.5           # 最小赛道宽度 (m)
  max_track_width: 4.0           # 最大赛道宽度 (m)
  max_repair_range: 15.0         # 仅 15m 内修复
  outlier_lateral_threshold: 5.0 # 横向离群阈值 (m)
```

---

## 开关状态速查表

| # | 模块 | 配置键 | 状态 |
|---|------|--------|------|
| 1 | CropBox ROI | `filters.use_cropbox` | ✅ ON |
| 2 | 强度滤波 | `filters.intensity.enable` | ❌ OFF |
| 3 | FGS 地面分割 | `ground_method: fgs` | ✅ ON |
| 4 | FGS 时序平滑 | `fgs.enable_temporal_smoothing` | ✅ ON |
| 5 | FGS 自适应 alpha | `fgs.enable_adaptive_alpha` | ✅ ON |
| 6 | 地面看门狗 | `ground_watchdog.enable` | ✅ ON |
| 7 | 柱状障碍物滤波 | `filters.obstacle_height.enable` | ✅ ON |
| 8 | 距离自适应体素 | `filters.distance_adaptive_voxel.enable` | ✅ ON |
| 9 | 普通体素 | `filters.voxel.enable` | ❌ OFF |
| 10 | 自适应体素 | `filters.adaptive_voxel.enable` | ❌ OFF |
| 11 | SOR | `filters.sor.enable` | ❌ OFF |
| 12 | 多帧累积 | `cluster.multi_frame.enable` | ❌ OFF |
| 13 | 欧式聚类 | `cluster.method: euclidean` | ✅ ON |
| 14 | FEC 加速 | `cluster.fec.enable` | ✅ ON |
| 15 | 自适应聚类大小 | `cluster.adaptive_size.enable` | ✅ ON |
| 16 | 置信度评分 | — | ✅ 始终 ON |
| 17 | 几何模型拟合 | `confidence.enable_model_fitting` | ✅ ON |
| 18 | 自适应 Y 轴 ROI | `roi.adaptive_y.enable` | ✅ ON |
| 19 | 赛道语义评分 | `confidence.track_semantic.enable` | ✅ ON |
| 20 | PCA 线性度惩罚 | `confidence.max_linearity` | ✅ ON (0.85) |
| 21 | 邻域硬剔除 | `track_semantic.min_neighbors_hard` | ✅ ON (0) |
| 22 | 近距离去重 | `dedup.enable` | ✅ ON |
| 23 | 时序跟踪器 | `tracker.enable` | ✅ ON |
| 24 | 拓扑修复 | `topology.enable` | ✅ ON |

**统计：** 18 项开启，6 项关闭。

---

## 与 FAILSAFE 的关系

`[high_speed_tracking] GENERAL FAILSAFE ACTIVATED!` 触发条件：

1. 车前方路径中点数 < 2（Delaunay 三角化生成的中线点不足）
2. 赛道尚未闭环
3. `general_failsafe` 配置为 true

**感知侧可能原因：**

- `roi.adaptive_y` 的 `far_y_half` 过窄 → 弯道远处锥桶被裁掉
- `tracker.confirm_frames` 过高 → 新锥桶确认延迟，规划看到的锥桶少
- `confidence.min_confidence_far` 过高 → 远处锥桶被过滤
- `obstacle_height.max_z_span` 过紧 → 坡度/姿态误差导致正常锥桶被误杀

**调试建议：**

```bash
# 对比开关前后效果
# 1. 关闭所有新增功能作为 baseline
roi.adaptive_y.enable: false
tracker.enable: false
topology.enable: false
confidence.track_semantic.enable: false

# 2. 逐项启用，观察 RViz 中锥桶数量变化
# 3. 重点关注弯道处和远处锥桶是否被误杀
```
