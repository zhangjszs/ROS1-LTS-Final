# 非视觉模式几何鲁棒性增强配置

**版本:** 1.0  
**日期:** 2026-02-11  
**模式:** 非视觉 / 纯几何降级  

---

## 1. 非视觉模式锁定

### 1.1 核心配置
```yaml
vision_mode:
  enabled: false                    # 视觉模式总开关（锁定为false）
  w_color_locked: 0.0              # 颜色权重锁定为0
  color_classification: NONE       # 颜色分类模式: NONE
```

### 1.2 强制几何降级
```yaml
geometry_fallback:
  enabled: true                     # 启用纯几何降级
  min_observations: 2               # 纯几何模式最小观测数
  use_geometric_features_only: true # 仅使用几何特征
```

**说明:**
- 所有颜色相关权重(w_color)强制为0.0
- 颜色分类完全禁用
- 数据关联仅基于几何特征
- 不影响主链路，纯几何降级独立运行

---

## 2. 几何鲁棒性增强模块

### 2.1 堆叠锥去重 (Stacked Cone Dedup)

**功能:** 检测并去除堆叠在同位置的锥桶，避免重复计数

**配置:**
```yaml
stacked_cone_dedup:
  enabled: true
  z_height_threshold: 0.3           # Z轴高度差阈值
  xy_distance_threshold: 0.3        # XY平面距离阈值
  max_cones_per_cell: 1             # 每个栅格最大锥桶数
  cell_size: 0.5                    # 栅格大小
  multi_layer_check: true           # 多层检测
  confidence_decay: 0.8             # 堆叠锥置信度衰减
```

**工作原理:**
1. 垂直分层检测：将点云按高度分层，检测垂直方向的堆叠
2. XY栅格去重：在XY平面划分栅格，每个栅格只保留一个锥桶
3. 置信度衰减：堆叠锥桶的置信度降低，优先使用非堆叠锥

**赛项特化:**
- **TrackDrive:** 精细分层(0.2m)，更多层数(4层)，应对高速密集场景
- **Acceleration:** 标准分层，宽松聚类，应对稀疏长距离场景
- **Skidpad:** 最精细分层(0.15m)，最多层数(5层)，应对八字密集场景

---

### 2.2 缺锥 Fallback (Missing Cone Fallback)

**功能:** 当检测到锥桶缺失时，使用几何预测进行补偿

**配置:**
```yaml
missing_cone_fallback:
  enabled: true
  max_interpolation_distance: 8.0   # 最大插值距离
  expected_spacing: 5.0             # 预期锥桶间距
  min_confidence_for_interpolation: 0.15
  max_consecutive_missing: 3        # 最大连续缺失数
  use_track_direction: true         # 利用赛道方向预测
  history_frames: 5                 # 历史帧数
```

**工作原理:**
1. 检测缺失：基于预期间距检测缺失的锥桶位置
2. 几何预测：使用历史轨迹和赛道方向预测缺失锥桶位置
3. 插值补偿：在缺失位置插入虚拟锥桶，保持路径连续性

**赛项特化:**
- **TrackDrive:** 长预测距离(12m)，较多连续预测(3)，应对频繁遮挡
- **Acceleration:** 最长预测距离(15m)，不使用曲率(直线场景)
- **Skidpad:** 圆弧插值，使用曲率，应对圆环几何

---

### 2.3 短路径抑制 (Short Path Suppression)

**功能:** 抑制由少量锥桶形成的短路径，避免不稳定轨迹

**配置:**
```yaml
short_path_suppression:
  enabled: true
  min_path_length: 3.0              # 最小有效路径长度
  min_cone_count: 3                 # 最小锥桶数量
  velocity_based_threshold: true    # 基于速度的阈值
  reject_single_cone_paths: true    # 拒绝单锥路径
```

**工作原理:**
1. 路径长度检查：路径长度必须超过阈值
2. 锥桶数量检查：至少包含最小数量的锥桶
3. 单锥拒绝：完全拒绝基于单个锥桶的路径

**赛项特化:**
- **TrackDrive:** 严格抑制(3帧确认，0.35置信度)，保证高速稳定性
- **Acceleration:** 宽松抑制(2帧确认，0.25置信度)，充分利用稀疏检测
- **Skidpad:** 最严格抑制(4帧确认，0.4置信度)，保证圆环完整性

---

### 2.4 回环稳定性 (Loop Closure Stability)

**功能:** 增强回环检测的稳定性，减少错误回环

**配置:**
```yaml
loop_closure:
  stability:
    enabled: true
    min_loop_length: 15.0          # 最小回环长度
    consistency_check_frames: 10    # 一致性检查帧数
    max_position_drift: 2.0        # 最大位置漂移
    max_heading_drift: 0.3         # 最大航向漂移
    use_geometric_verification: true
    min_inlier_ratio: 0.6
```

**工作原理:**
1. 几何验证：检查回环的相对几何一致性
2. 连续性检查：多帧一致性检查，避免瞬态错误
3. 漂移限制：限制位置和航向漂移，保证回环质量

**赛项特化:**
- **TrackDrive:** 中等特征寿命(8帧)，频繁回环场景
- **Acceleration:** 较少使用回环(直线场景)
- **Skidpad:** 最长特征寿命(10帧)，最高重识别门槛(0.8)

---

## 3. 配置文件位置

### 3.1 定位模块配置
- **基础配置:** `src/localization_ros/config/location_common.yaml`
- **TrackDrive覆盖:** `src/localization_ros/config/location_track.yaml`
- **Acceleration覆盖:** `src/localization_ros/config/location_accel.yaml`
- **Skidpad覆盖:** `src/localization_ros/config/location_skidpad.yaml`

### 3.2 感知模块配置
- **基础配置:** `src/perception_ros/config/lidar_base.yaml`
- **TrackDrive覆盖:** `src/perception_ros/config/lidar_track.yaml`
- **Acceleration覆盖:** `src/perception_ros/config/lidar_accel.yaml`
- **Skidpad覆盖:** `src/perception_ros/config/lidar_skidpad.yaml`

---

## 4. 验证方法

### 4.1 非视觉模式验证
```bash
# 检查w_color是否为0
rosparam get /location_node/fg/w_color
# 期望输出: 0.0

# 检查颜色分类是否禁用
rosparam get /vision_mode/enabled
# 期望输出: false
```

### 4.2 几何鲁棒性验证
```bash
# 检查堆叠锥去重
rosparam get /stacked_cone_dedup/enabled
# 期望输出: true

# 检查缺锥fallback
rosparam get /missing_cone_fallback/enabled
# 期望输出: true

# 检查短路径抑制
rosparam get /short_path_suppression/enabled
# 期望输出: true
```

---

## 5. 性能指标

### 5.1 预期改进
- **堆叠锥去重:** 减少10-20%的重复锥桶计数
- **缺锥Fallback:** 提高5-15%的路径连续性
- **短路径抑制:** 减少30-50%的不稳定轨迹
- **回环稳定性:** 减少50-80%的错误回环

### 5.2 监控指标
```bash
# 锥桶计数稳定性
rostopic echo /localization/cone_map | grep -c "cone"

# 路径连续性
rostopic hz /planning/pathlimits

# 回环频率
rostopic echo /localization/loop_closure_events
```

---

## 6. 故障排除

### 6.1 堆叠锥过度去重
**症状:** 有效锥桶被错误去除
**解决:** 
```bash
# 放宽XY阈值
rosparam set /stacked_cone_dedup/xy_distance_threshold 0.4
```

### 6.2 缺锥预测过多
**症状:** 虚拟锥桶过多导致路径漂移
**解决:**
```bash
# 降低预测置信度
rosparam set /missing_cone_fallback/prediction_confidence 0.15
# 减少最大连续预测
rosparam set /missing_cone_fallback/max_consecutive_predictions 2
```

### 6.3 短路径抑制过强
**症状:** 有效路径被抑制
**解决:**
```bash
# 降低最小路径长度
rosparam set /short_path_suppression/min_path_length 2.0
# 降低最小锥桶数
rosparam set /short_path_suppression/min_cone_count 2
```

---

## 7. 版本历史

| 版本 | 日期 | 修改 |
|------|------|------|
| 1.0 | 2026-02-11 | 初始版本，非视觉模式锁定 + 几何鲁棒性增强 |

---

**注意:** 本文档描述的配置基于纯几何特征，不依赖任何视觉信息。所有参数针对非视觉模式优化，确保在无相机输入时系统仍能稳定运行。
