
# LiDAR数据处理管道系统性分析报告

## 1. 管道总体架构

```
[Velodyne Points] → [输入防御] → [畸变补偿] → [ROI+强度滤波] → [地面分割] → [后地面滤波]
    → [多帧累积] → [距离分段聚类] → [特征提取] → [置信度评分] → [时序跟踪] → [拓扑修复] → [输出发布]
```

核心文件映射：
- 数据流编排: `src/perception_ros/src/lidar_cluster_ros.cpp` (ROS包装器)
- 算法核心: `src/perception_core/src/lidar_cluster_core.cpp` + `utility.cpp`
- 置信度评分: `src/perception_core/src/confidence_scorer.cpp`
- 特征提取: `src/perception_core/src/cluster_feature_extractor.cpp`
- 模型拟合: `src/perception_core/src/cone_model_fitter.cpp`

---

## 2. 分阶段参数必要性分析

### Stage 1: 输入接收与预处理 (Input Guard + Distortion Compensation)

| 参数 | 必要性 | 说明 |
|------|--------|------|
| `input_guard/enable` | ✅ 必要 | 防御空点云、超大点云、NaN/Inf点，生产必需 |
| `input_guard/max_points` | ✅ 必要 | 500000点上限合理，防止内存爆炸 |
| `input_guard/filter_invalid_points` | ✅ 必要 | 清除NaN/Inf，PCL算法前置条件 |
| `imu/distortion/enable` | ⚠️ 条件必要 | 高速场景(>10m/s)强烈建议启用，当前`false` |
| `imu/distortion/scan_period` | ⚠️ 条件必要 | 需与传感器实际周期匹配(VLP-32: ~0.1s) |

**评估**: 输入防御3参数均为P0必要。畸变补偿当前关闭，在TrackDrive高速场景下是**明显短板**——10m/s车速下0.1s扫描周期内车辆移动1m，不补偿会导致远处点云畸变。

---

### Stage 2: ROI裁剪与初始滤波 (PassThroughROI)

| 参数 | 必要性 | 说明 |
|------|--------|------|
| `roi/mode` | ✅ 必要 | 模式切换(track/accel/skidpad)，决定参数覆盖 |
| `roi/z_min/z_max` | ✅ 必要 | Z轴裁剪，排除地面以下和过高物体 |
| `roi/track/x_min...y_max` | ✅ 必要 | 赛道范围约束，减少计算量 |
| `roi/adaptive_y/enable` | ✅ 必要 | 远处收窄Y范围，有效抑制侧向墙/护栏 |
| `roi/adaptive_y/near_y_half` | ✅ 必要 | 近处全宽度检测 |
| `roi/adaptive_y/far_y_half` | ✅ 必要 | 远处收窄值，当前track: 4.0m |
| `roi/adaptive_y/ramp_start_x` | ✅ 必要 | 收窄起始距离，当前15.0m |
| `roi/center_exclusion/enable` | ✅ 必要 | 中心线假锥抑制，高速场景关键 |
| `filters/intensity/enable` | ✅ 必要 | 强度滤波，抑制草/灌木低反射噪声 |
| `filters/intensity/min_intensity` | ✅ 必要 | 当前12.0，已优化至较低水平 |
| `filters/use_cropbox` | ⚠️ 建议保留 | CropBox比3次PassThrough快，但参数影响小 |

**评估**: ROI参数体系设计合理。adaptive_y + center_exclusion 是远距离召回率的关键保障。

---

### Stage 3: 地面分割 (Ground Segmentation)

| 参数 | 必要性 | 说明 |
|------|--------|------|
| `ground_method` | ✅ 必要 | ransac/patchworkpp/fgs三选一 |
| `force_fgs_fast_path` | ⚠️ 冗余 | 强制覆盖ground_method，可合并 |
| `sensor_height` | ✅ 必要 | 影响地面高度估计 |
| `ground_watchdog/*` | ✅ 必要 | 监控地面分割耗时，诊断用 |

**FGS参数 (当前主路径)**:
| 参数 | 必要性 | 说明 |
|------|--------|------|
| `fgs/num_sectors` (32) | ✅ 必要 | 角度分辨率，影响地面模型精度 |
| `fgs/num_bins` (80) | ✅ 必要 | 距离分辨率，当前80bin覆盖80m=1m/bin |
| `fgs/th_ground` (0.08) | ✅ 必要 | 核心地面阈值 |
| `fgs/th_ground_far` (0.15) | ✅ 必要 | 远处放宽，适应点云稀疏 |
| `fgs/far_distance` (20.0) | ✅ 必要 | 远近分界 |
| `fgs/max_slope` (0.3) | ✅ 必要 | 坡度约束 |
| `fgs/min_normal_z` (0.85) | ✅ 必要 | 法向量约束，排除竖直平面 |
| `fgs/enable_temporal_smoothing` | ✅ 必要 | 帧间EMA，抑制地面闪烁 |
| `fgs/temporal_alpha` (0.5) | ✅ 必要 | EMA系数 |
| `fgs/enable_adaptive_alpha` | ⚠️ 建议保留 | S弯场景有用，但track场景直道多 |
| `fgs/use_lowest_n_mean` | ⚠️ 较低 | lowest-N均值 vs min_z，差异不大 |
| `fgs/lowest_n` (3) | ⚠️ 较低 | 同上 |
| `fgs/enable_sector_smoothing` | ⚠️ 较低 | 相邻扇区平滑，效果有限 |
| `fgs/enable_refinement` | ❌ 冗余 | 当前false，二次拟合开销大收益小 |
| `fgs/max_segments_per_sector` (4) | ⚠️ 较低 | 线段生长控制，正常场景不触发 |
| `fgs/segment_merge_dist` (0.15) | ⚠️ 较低 | 同上 |
| `fgs/ground_below_factor` (1.5) | ⚠️ 较低 | 下方阈值放宽，特殊场景才需 |
| `fgs/use_neighbor_model` | ⚠️ 较低 | 无效扇区补全，极少触发 |

**RANSAC参数 (备用路径)**:
- 当前被force_fgs_fast_path覆盖，实际上**未使用**
- 17个RANSAC参数成为死参数

**Patchwork++参数 (备用路径)**:
- 同理，当前被强制使用FGS
- 约25个Patchwork++参数成为死参数

**评估**:
- FGS主路径设计合理，核心参数8个足够。
- `force_fgs_fast_path=true` 导致RANSAC和Patchwork++的全部参数成为**死代码/死参数**。
- 建议：删除force_fgs_fast_path，明确选择ground_method=fgs；或清理未使用方法的参数加载代码。

---

### Stage 4: 后地面滤波 (PostGroundFilter)

| 参数 | 必要性 | 说明 |
|------|--------|------|
| `filters/obstacle_height/enable` | ✅ 必要 | 柱状障碍物滤波，抑制树/墙/护栏 |
| `filters/obstacle_height/grid_size` (0.5) | ✅ 必要 | 栅格大小，与锥桶尺寸匹配 |
| `filters/obstacle_height/max_z_span` (0.3) | ✅ 必要 | z跨度阈值，核心判别参数 |
| `filters/obstacle_height/min_points_to_judge` (4) | ✅ 必要 | 最少判定点数 |
| `filters/obstacle_height/min_distance` (5.0) | ✅ 必要 | 生效距离下限 |
| `filters/distance_adaptive_voxel/enable` | ⚠️ 条件必要 | 当前YAML中enable=true但代码默认值false |
| `filters/distance_adaptive_voxel/near_leaf` (0.08) | ⚠️ 条件必要 | 近处降采样，减少计算量 |
| `filters/distance_adaptive_voxel/far_leaf` (0.03) | ⚠️ 条件必要 | 远处小体素保精度，但3cm可能造成过稀疏 |
| `filters/distance_adaptive_voxel/dist_threshold` (10.0) | ⚠️ 条件必要 | 远近分界 |
| `filters/voxel/enable` | ❌ 冗余 | 被distance_adaptive_voxel替代 |
| `filters/voxel/leaf_size` | ❌ 冗余 | 同上 |
| `filters/adaptive_voxel/enable` | ❌ 冗余 | 同上 |
| `filters/sor/enable` | ❌ 建议移除 | 当前false，SOR对锥桶边缘有损害 |
| `filters/sor/mean_k` | ❌ 建议移除 | 同上 |

**评估**:
- obstacle_height 是高速场景P0参数，有效抑制远处护栏。
- 体素滤波存在**三重冗余**: voxel / adaptive_voxel / distance_adaptive_voxel，代码逻辑是互斥if-else，但配置层面混乱。
- SOR对锥桶检测有害（会移除锥桶边缘稀疏点），建议从配置中移除。

---

### Stage 5: 多帧累积 (AccumulateFrames)

| 参数 | 必要性 | 说明 |
|------|--------|------|
| `cluster/multi_frame/enable` | ⚠️ 问题参数 | YAML中false，代码默认true，加载代码存在但不一致 |
| `cluster/multi_frame/num_frames` (2) | ⚠️ 问题参数 | 累积帧数 |
| `cluster/multi_frame/max_distance` (10.0) | ⚠️ 问题参数 | 仅远处点累积 |

**评估**:
- `multi_frame` 的问题是**默认值体系不一致**：`lidar_base.yaml` 中显式设为 `false`，但 `lidar_cluster_core.hpp:365` 的代码默认值为 `true`，且 `lidar_cluster_ros.cpp:770` 的加载逻辑存在。由于 YAML 显式值会覆盖代码默认值，当前运行时行为是确定的（即 `false`），但维护者容易产生误判。
- 多帧累积对远处召回有帮助，但会引入**时延和鬼影**（运动锥桶拖尾）。
- 建议：统一代码默认值与 YAML 显式值（均设为 `false`），消除语义冲突。

---

### Stage 6: 聚类 (Clustering)

| 参数 | 必要性 | 说明 |
|------|--------|------|
| `cluster/method` (euclidean) | ✅ 必要 | 聚类方法选择 |
| `cluster/distance_segments` | ✅ 必要 | 距离分段阈值 |
| `cluster/cluster_tolerance` | ✅ 必要 | 各段聚类距离，核心参数 |
| `cluster/adaptive_size/enable` | ✅ 必要 | 自适应聚类大小 |
| `cluster/adaptive_size/near_min/max_size` | ✅ 必要 | 近距离聚类大小限制 |
| `cluster/adaptive_size/far_min/max_size` | ✅ 必要 | 远距离聚类大小限制 |
| `cluster/fec/enable` | ✅ 必要 | FEC快速聚类替代PCL原版 |
| `cluster/fec/quality` (0.3) | ✅ 必要 | 速度-精度权衡 |
| `cluster/min_cluster_size` (1) | ⚠️ 较低 | 被adaptive_size覆盖 |
| `cluster/max_cluster_size` (50) | ⚠️ 较低 | 被adaptive_size覆盖 |
| `cluster/dbscan/*` (7个参数) | ❌ 死参数 | method=euclidean时全部未使用 |
| `cluster/vlp16/*` (4个参数) | ❌ 死参数 | sensor_model=32时未使用 |
| `cluster/point_clip/*` (2个参数) | ❌ 死参数 | 仅在skidpad+use_point_clip时使用 |

**评估**:
- 距离分段聚类是核心设计，参数合理。
- DBSCAN全部参数、VLP-16参数、point_clip参数在TrackDrive模式下均为**死参数**。
- adaptive_size与fixed size(min/max_cluster_size)存在功能重叠。

---

### Stage 7: 置信度评分 (Confidence Scoring) — 最复杂阶段

| 参数 | 必要性 | 说明 |
|------|--------|------|
| `confidence/min_height` (0.10) | ✅ 必要 | 高度下限，抑制低矮噪声 |
| `confidence/max_height` (0.55) | ✅ 必要 | 高度上限，抑制墙/护栏碎片 |
| `confidence/min_area` (0.01) | ✅ 必要 | 面积下限 |
| `confidence/max_area` (0.35) | ✅ 必要 | 面积上限，远处放宽 |
| `confidence/max_box_altitude` (0.05) | ✅ 必要 | 地面高度约束 |
| `confidence/min_aspect_ratio` (0.9) | ✅ 必要 | 纵横比 |
| `confidence/min_verticality` (0.65) | ✅ 必要 | 垂直度，当前已优化降低 |
| `confidence/max_linearity` (0.80) | ✅ 必要 | 线性度上限，抑制墙/栏杆 |
| `confidence/min_density_near` (60.0) | ✅ 必要 | 近处密度门槛 |
| `confidence/min_density_far` (8.0) | ✅ 必要 | 远处密度门槛，已大幅放宽 |
| `confidence/distance_threshold` (5.0) | ✅ 必要 | 近远距离分界 |
| `confidence/min_intensity_mean` (45.0) | ✅ 必要 | 强度门槛，已优化放宽 |
| `confidence/weight_size` (0.20) | ✅ 必要 | 尺寸权重 |
| `confidence/weight_shape` (0.30) | ✅ 必要 | 形状权重 |
| `confidence/weight_density` (0.20) | ✅ 必要 | 密度权重 |
| `confidence/weight_intensity` (0.20) | ✅ 必要 | 强度权重 |
| `confidence/weight_position` (0.10) | ✅ 必要 | 位置权重 |
| `confidence/enable_model_fitting` | ✅ 必要 | 3D锥桶模型拟合开关 |
| `confidence/model_fit_bonus` (0.65) | ✅ 必要 | 拟合成功奖励，已提高 |
| `confidence/model_fit_penalty` (0.06) | ✅ 必要 | 拟合失败惩罚，已降低 |
| `confidence/min_confidence_near` (0.20) | ✅ 必要 | 近处门槛 |
| `confidence/min_confidence_far` (0.45) | ✅ 必要 | 远处门槛，核心调参点 |
| `confidence/confidence_ramp_start` (8.0) | ✅ 必要 | ramp起始 |
| `confidence/confidence_ramp_end` (40.0) | ✅ 必要 | ramp结束，已优化 |
| `confidence/track_semantic/enable` | ✅ 必要 | 邻域语义评分开关 |
| `confidence/track_semantic/weight` (0.10) | ✅ 必要 | 语义权重 |
| `confidence/track_semantic/expected_track_width` (3.5) | ✅ 必要 | 赛道宽度先验 |
| `confidence/track_semantic/expected_cone_spacing` (5.0) | ✅ 必要 | 锥桶间距先验 |
| `confidence/track_semantic/spacing_tolerance` (2.0) | ✅ 必要 | 间距容差 |
| `confidence/track_semantic/width_tolerance` (1.0) | ✅ 必要 | 宽度容差 |
| `confidence/track_semantic/isolation_radius` (8.0) | ✅ 必要 | 邻居搜索半径 |
| `confidence/track_semantic/min_neighbors_hard` (0) | ⚠️ 建议调整 | 0表示不禁用，设1-2可过滤孤立噪声 |
| `confidence/track_semantic/max_isolation_distance` (12.0) | ✅ 必要 | 最大孤立距离 |

**评估**:
- 置信度系统是当前最成熟的子系统，参数设计合理且经过多轮调优。
- 5维权重和+距离自适应+模型拟合+语义上下文构成了**四层防御体系**。
- track_semantic/min_neighbors_hard=0 是一个**风险点**：完全孤立的检测仍可通过，建议设为1。

---

### Stage 8: 时序跟踪 (Cone Tracker)

| 参数 | 必要性 | 说明 |
|------|--------|------|
| `tracker/enable` | ✅ 必要 | 总开关，track模式已启用 |
| `tracker/association_threshold` (2.0) | ✅ 必要 | 关联阈值，高速场景需较大值 |
| `tracker/confirm_frames` (4) | ✅ 必要 | 确认帧数，平衡延迟与稳定性 |
| `tracker/delete_frames` (5) | ✅ 必要 | 删除帧数 |
| `tracker/process_noise` (0.1) | ⚠️ 较低 | 卡尔曼Q矩阵，对匀速模型影响小 |
| `tracker/measurement_noise` (0.05) | ⚠️ 较低 | 卡尔曼R矩阵，同上 |
| `tracker/only_output_confirmed` | ✅ 必要 | 仅输出确认锥桶，抑制闪烁 |
| `tracker/confirmed_confidence_boost` (0.1) | ✅ 必要 | 确认后置信度加成 |

**评估**: Tracker是稳定性关键。process_noise/measurement_noise对简单匀速模型影响有限，可合并为单一`tracking_smoothness`参数。

---

### Stage 9: 拓扑修复 (Topology Repair)

| 参数 | 必要性 | 说明 |
|------|--------|------|
| `topology/enable` | ✅ 必要 | 总开关 |
| `topology/max_same_side_spacing` (5.0) | ✅ 必要 | 同侧最大间距，决定插值距离 |
| `topology/min_track_width` (2.5) | ✅ 必要 | 最小赛道宽度 |
| `topology/max_track_width` (4.0) | ✅ 必要 | 最大赛道宽度 |
| `topology/max_repair_range` (15.0) | ✅ 必要 | 修复最大距离 |
| `topology/outlier_lateral_threshold` (5.0) | ⚠️ 较低 | 横向离群阈值，使用频率低 |
| `topology/interpolated_confidence` (0.2) | ✅ 必要 | 插值锥桶置信度 |

**评估**: Topology Repair对连续性至关重要。参数精简合理。

---

### Stage 10: 去重 (Deduplication)

| 参数 | 必要性 | 说明 |
|------|--------|------|
| `dedup/enable` | ✅ 必要 | 总开关 |
| `dedup/radius` (0.6) | ✅ 必要 | 抑制半径 |

**评估**: 简单有效。

---

## 3. 冗余/死参数识别

### 3.1 完全未使用的参数组 (YAML存在但代码无加载)

以下参数在 `lidar_base.yaml` 和 `lidar_track.yaml` 中定义，但在 `lidar_cluster_ros.cpp` 的 `loadParams()` 中**没有任何加载代码**，属于纯粹的配置僵尸：

| 参数组 | 参数数量 | 状态 |
|--------|---------|------|
| `occlusion_handling/*` | ~12个 | ❌ 纯死参数 |
| `short_detection_suppression/*` | ~8个 | ❌ 纯死参数 |
| `geometric_consistency/*` | ~12个 | ❌ 纯死参数 |
| `loop_closure_optimization/*` | ~10个 | ❌ 纯死参数 |
| `vision_independent_mode/*` | ~4个 | ❌ 纯死参数 |

**总计约 46 个纯死参数**。

**例外：`stacked_cone_detection/*` 为部分接线**

`stacked_cone_detection` 不是纯死参数。在 `lidar_cluster_ros.cpp:972-981` 中，有 **5 个参数**已加载到 `config_.dedup.*` 路径：
- `stacked_cone_detection/enabled`
- `stacked_cone_detection/vertical_layers/layer_height`
- `stacked_cone_detection/vertical_layers/max_layers`
- `stacked_cone_detection/identification/max_xy_distance`
- `stacked_cone_detection/identification/height_variation_threshold`

但 YAML 中同组还定义了约 **10 个参数**（如 `vertical_layers/overlap_threshold`、`identification/min_points_per_layer`、`deduplication/*`、`near_range/*` 等）在 `loadParams()` 中**无任何加载代码**，处于悬空状态。

**建议**：保留已接线的 5 个参数，删除 YAML 中悬空的 10 个参数，或补充其加载代码。

### 3.2 条件死参数 (特定模式下未使用)

| 参数组 | 触发条件 | 当前状态 |
|--------|---------|---------|
| `ransac/*` (17个) | `ground_method="ransac"` | force_fgs_fast_path=true，未使用 |
| `patchworkpp/*` (25个) | `ground_method="patchworkpp"` | 同上，未使用 |
| `cluster/dbscan/*` (7个) | `cluster.method="dbscan"` | 当前euclidean，未使用 |
| `cluster/vlp16/*` (4个) | `sensor_model=16` | 当前32，未使用 |
| `cluster/point_clip/*` (2个) | `roi.use_point_clip=true` | 当前false，未使用 |
| `filters/sor/*` (3个) | `filters.sor.enable=true` | 当前false，未使用 |
| `filters/voxel/*` (2个) | `filters.voxel.enable=true` | 被distance_adaptive覆盖 |
| `filters/adaptive_voxel/*` (3个) | `filters.adaptive_voxel.enable=true` | 被distance_adaptive覆盖 |

**条件死参数总计约 63 个**。

### 3.3 功能重复参数

| 重复组 | 重复参数 | 建议 |
|--------|---------|------|
| 体素滤波 | voxel / adaptive_voxel / distance_adaptive_voxel | 只保留distance_adaptive_voxel，删除另外两个 |
| 聚类大小 | adaptive_size vs min/max_cluster_size | adaptive_size.enable=true时，fixed size被覆盖，保留adaptive_size |
| 传感器高度 | 顶层sensor_height vs fgs/sensor_height | 合并为一个参数 |
| 道路类型 | roi.mode vs road_type | road_type为兼容保留，可统一为roi.mode |

---

## 4. 管道必要性与合理性评估

### 4.1 整体架构评分

| 维度 | 评分 | 说明 |
|------|------|------|
| 模块化 | ⭐⭐⭐⭐⭐ | Core+ROS Wrapper分离，测试友好 |
| 可配置性 | ⭐⭐⭐☆☆ | 参数过多且30%为死参数，认知负担重 |
| 实时性 | ⭐⭐⭐⭐☆ | FGS<5ms，整体~15ms，满足10Hz需求 |
| 远距召回 | ⭐⭐⭐⭐☆ | 30-50m约2.25 cones/frame，仍有提升空间 |
| 稳定性 | ⭐⭐⭐⭐⭐ | Tracker+Topology双重时序保障 |

### 4.2 各阶段合理性

| 阶段 | 合理性 | 关键问题 |
|------|--------|---------|
| 输入防御 | ✅ 合理 | 三层防御（空/超大/NaN）必要且轻量 |
| 畸变补偿 | ⚠️ 不足 | 当前关闭，高速场景应启用 |
| ROI+强度滤波 | ✅ 合理 | adaptive_y设计精妙，有效抑制侧向干扰 |
| 地面分割 | ✅ 合理 | FGS O(n)复杂度，<5ms，适合实时 |
| 柱状障碍物滤波 | ✅ 合理 | 远处护栏抑制的关键 |
| 体素降采样 | ⚠️ 混乱 | 三选一逻辑，配置层面不清晰 |
| 多帧累积 | ⚠️ 矛盾 | YAML与代码默认值不一致 |
| 距离分段聚类 | ✅ 合理 | 远近不同tolerance是标准做法 |
| 特征提取 | ✅ 合理 | 5维特征覆盖几何/形状/密度/强度/位置 |
| 置信度评分 | ✅ 优秀 | 四层防御（基础+拟合+语义+距离自适应） |
| 时序跟踪 | ✅ 合理 | 卡尔曼+确认机制，有效抑制闪烁 |
| 拓扑修复 | ✅ 合理 | 插值填补缺失，保证连续性 |
| 去重 | ✅ 合理 | 简单有效 |

### 4.3 性能瓶颈分析

从代码结构判断（无profiling数据）：
1. **地面分割**: FGS约2-5ms，主瓶颈之一
2. **聚类**: 欧式聚类O(n log n) + KdTree构建，数据量大时耗时
3. **模型拟合**: RANSAC 100次迭代 + PCA + LS，每个候选簇都执行，当cluster数量多时（如>50）可能成为瓶颈
4. **特征提取**: PCA计算3×3协方差矩阵特征值，较快

---

## 5. 优化策略建议

### 5.1 立即执行 (P0 - 安全与正确性)

1. **清理死参数**
   - 从YAML中删除纯死参数组：`occlusion_handling/*`、`short_detection_suppression/*`、`geometric_consistency/*`、`loop_closure_optimization/*`、`vision_independent_mode/*`（约46个）
   - 从YAML中删除 `stacked_cone_detection` 里悬空的约10个参数（保留已接线的5个）
   - 建议优先**删除YAML中的死参数**，减少配置噪音

2. **统一多帧累积默认值**
   - `cluster/multi_frame/enable`: 统一YAML和代码默认值为`false`
   - 若未来启用，需解决运动拖尾问题

3. **启用畸变补偿**
   - TrackDrive高速场景建议启用 `imu/distortion/enable`
   - 需要校准IMU-LiDAR外参

### 5.2 短期优化 (P1 - 效率与召回)

4. **简化体素滤波配置**
   - 删除 `filters/voxel/*` 和 `filters/adaptive_voxel/*`
   - 只保留 `filters/distance_adaptive_voxel/*`
   - 将默认enable统一为true

5. **清理条件死参数**
   - 删除 `cluster/dbscan/*` 和 `cluster/vlp16/*` 参数加载代码
   - 或：在launch文件中注释掉这些参数，注明"当前未使用"

6. **模型拟合性能优化**
   - 对远距离(>30m)且点数<8的簇，跳过模型拟合（拟合不可靠）
   - 或对低置信度候选(<0.3)跳过拟合，节省计算
   - 预计可节省20-30%的拟合计算量

7. **track_semantic调优**
   - 将 `min_neighbors_hard` 从0调整为1
   - 完全孤立的检测（0邻居）在8m半径内几乎不可能是真锥桶
   - 可进一步抑制中心线假锥

### 5.3 中期优化 (P2 - 架构改进)

8. **参数层级重构**
   - 当前参数分三层：base.yaml → mode overlay → code default
   - 建议改为两层：mode-specific full config → vehicle/local overlay
   - 减少参数继承的复杂度

9. **远距离召回率专项**
   - 30-50m当前约2.25 cones/frame，是主要短板
   - 分析：VLP-32在50m处角分辨率约0.17°，锥桶直径~19cm，理论可探测但点极少(1-2点)
   - 策略：
     a. 进一步放宽 `min_density_far` 至 5.0 或更低
     b. 对>30m簇，降低 `min_verticality` 至 0.55
     c. 考虑启用多帧累积（权衡时延）
     d. 提升模型拟合对2-3点簇的鲁棒性（当前min_points_for_fitting=5）

10. **置信度评分动态学习**
    - 当前权重是静态的，可考虑在线适应：
    - 基于tracker确认历史，统计真锥/假锥的特征分布
    - 动态调整各维度权重（如某场景intensity特别可靠则提升其权重）

### 5.4 长期方向 (P3 - 算法升级)

11. **端到端学习替代手工规则**
    - 当前置信度系统是手工设计的5维加权+硬阈值
    - 可用小MLP（输入5维特征+距离，输出置信度）替代
    - 在rosbag数据上训练，保持推理延迟<1ms

12. **多传感器融合优化**
    - 当前vision融合通过`fusion/*`参数配置
    - 远距离(>40m)视觉几乎无效，近处(<15m)视觉颜色可靠
    - 建议按距离段切换融合策略

---

## 6. 参数精简建议

**当前参数规模**:
- YAML可见参数: ~250个（含嵌套）
- 代码加载参数: 273个加载调用
- 实际有效参数: 约120个
- 纯死参数: ~46个
- 条件死参数: ~63个
- 功能重复参数: ~11个
- 悬空参数（部分接线组）: ~10个（stacked_cone_detection 中未加载部分）
- **死/冗余参数总计: ~130个（~52%冗余率）**

**精简目标**: 将有效参数控制在80个以内

| 动作 | 可删除参数数 | 优先级 |
|------|-------------|--------|
| 删除纯死参数组 + stacked_cone悬空部分 | ~56 | P0 |
| 删除未使用方法的参数加载 | ~45 | P1 |
| 合并重复体素滤波 | ~5 | P1 |
| 简化tracker噪声模型 | ~1 | P2 |
| 删除legacy参数(road_type等) | ~3 | P2 |

**预计精简后**: 有效参数 ~80个，死参数 ~10个，冗余率降至 **~11%**。

---

---

## Appendix A: Task #20 Far-Range Threshold Grid Experiment (2026-04-25)

### A.1 Experiment Design

**Objective**: Quantify the marginal gain from far-range threshold tuning (min_neighbors_hard, min_confidence_far, cluster_tolerance_far_profile) on 30-50m cone recall.

**Matrix**: 2 × 3 × 2 = 12 runs
- `min_neighbors_hard`: {1, 2}
- `min_confidence_far`: {0.40, 0.45, 0.50}
- `cluster_tolerance_far_profile`: {current, relaxed_far}
- `cluster_tolerance` values:
  - current: [0.15, 0.25, 0.28, 0.35, 0.38, 0.40, 0.42]
  - relaxed_far: [0.15, 0.25, 0.28, 0.35, 0.42, 0.45, 0.45]

**Bag**: `~/rosbag/track.bag` (low-speed, ~2.5 m/s), start offset 30s, duration 60s per run.

**Key Metrics**:
- `30-50m/frame` — primary target
- `spike_rate` = std/mean of cones/frame — stability proxy
- `short_path%` — planning degradation indicator
- `mean_speed` — should not drop

### A.2 Results Summary

| Best Candidates | 30-50m/frame | spike_rate | short_path% | mean_speed |
|-----------------|--------------|------------|-------------|------------|
| n1_cf50_current | 1.91 | 0.731 | 0.32% | 2.59 |
| n2_cf45_current | 2.10 | 0.727 | 0.68% | 2.50 |
| **n1_cf45_current (baseline)** | **2.10** | **0.712** | **0.0%** | **2.47** |
| n2_cf50_relaxed_far | 2.23 | 0.708 | 1.08% | 2.47 |

- **Maximum improvement in 30-50m/frame**: ~7% (n2_cf50_relaxed_far vs baseline)
- **>50m/frame**: 0.0 across all 12 runs (hard-limited by `roi/track/x_max: 50`)
- **spike_rate increase**: <3% relative for best candidates
- **short_path%**: Baseline 0.0%; some candidates degraded to 0.3-1.1%
- **Mean speed**: 2.47-2.59 m/s, no significant variation

### A.3 Distortion Compensation Verification

After the grid experiment, enabled IMU-based deskew (`imu/enable: true`, `imu/distortion/enable: true`) with:
- IMU topic: `/sensors/ins` (fixed from legacy `/pbox_pub/Ins`)
- `point_time_source: angle` (VLP-32 point cloud has no `time` field)

| State | 30-50m/frame | short_path% | delta_std | mean_speed |
|-------|--------------|-------------|-----------|------------|
| Deskew OFF | 2.10 | 0.0% | 0.0645 | 2.47 |
| Deskew ON | 2.10 | 0.97% | 0.1075 | 2.47 |

**Conclusion**: At ~2.5 m/s (max displacement ~0.25m/scan), deskew provides **zero measurable benefit** in far-range recall and **increases control jitter by 67%** (delta_std 0.0645 → 0.1075). Likely cause: IMU velocity noise dominates over actual motion displacement at this speed.

**Decision**: Keep deskew implementation in code but disable by default (`imu/enable: false`, `imu/distortion/enable: false`). Re-enable when high-speed bag (>8 m/s) is available for validation.

### A.4 Final Parameter State (Task #20 Complete)

Kept baseline `n1_cf45_current` as production default:
- `track_semantic/min_neighbors_hard: 1`
- `confidence/min_confidence_far: 0.45`
- `cluster/cluster_tolerance: [0.15, 0.25, 0.28, 0.35, 0.38, 0.40, 0.42]` (current profile)

Bug fix retained:
- `imu/topic: /sensors/ins`

Deskew state:
- `imu/enable: false`
- `imu/distortion/enable: false`

### A.5 Key Takeaways

1. **Threshold tuning has hit diminishing returns**: 12-run grid yielded <10% improvement in 30-50m recall. The bottleneck is not clustering tolerance or confidence threshold.
2. **ROI limits >50m detection**: `x_max: 50` makes all >50m experiments impossible. If >50m is needed, must lift ROI first.
3. **Distortion compensation requires high-speed validation**: At low speed, IMU noise dominates signal. Do not enable without a high-speed bag.
4. **Next frontier**: 30-50m recall likely requires sensor-level or algorithmic changes (e.g., multi-frame accumulation, deep learning-based detection, or higher-resolution LiDAR), not parameter tweaks.

---

## 7. 结论

该LiDAR处理管道在**架构层面设计合理**，Core+ROS分离、事件驱动、多级过滤均符合FSD实时感知需求。核心算法（FGS地面分割、距离分段聚类、四层置信度评分、Tracker+Topology时序稳定）经过多轮迭代已相对成熟。

**主要问题**在于：
1. **配置债务**: 约52%的参数为死参数或冗余参数，严重影响调参效率
2. **畸变补偿缺失**: 高速场景未启用，影响远距精度（已在代码实现，验证后默认关闭）
3. **30-50m召回瓶颈**: 传感器物理限制+保守阈值导致远距召回不足；Task #20证明参数调优边际收益已耗尽

**建议优先执行 P0 清理 + P1 简化**，再针对远距离召回进行专项调优。
