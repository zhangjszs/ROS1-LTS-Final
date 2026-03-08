# 定位系统接口冻结文档 (Localization Interface Freeze)

> **版本**: v1.0
> **状态**: DRAFT
> **适用范围**: `localization_ros` / `localization_core` 模块
> **冻结含义**: 下游模块（planning、control、visualization）可依赖本文档中定义的话题名、消息类型、坐标系和单位，后续变更需走 breaking-change 流程。

---

## 1. 系统边界

```
                    ┌─────────────────────────────┐
  /pbox_pub/Ins ──► │                             │──► localization/car_state
  (HUAT_InsP2)      │                             │──► localization/cone_map
                    │      location_node          │──► localization/global_map
  perception/       │                             │──► localization/pose
  lidar_cluster/ ──►│  backend: mapper | fg       │──► localization/odom
  detections        │                             │──► TF: world → base_link
  (ConeDetections)  └─────────────────────────────┘
```

---

## 2. 输入接口

### 2.1 INS 输入 — `HUAT_InsP2`

| 属性 | 值 |
|------|-----|
| **逻辑话题** | `sensors/ins` |
| **实际话题** | `/pbox_pub/Ins` (通过 launch remap) |
| **消息类型** | `autodrive_msgs/HUAT_InsP2` |
| **发布频率** | ~100 Hz |
| **队列长度** | 10 |

#### 字段契约

| 字段 | 类型 | 单位 | 坐标系 | 说明 |
|------|------|------|--------|------|
| `header.stamp` | `time` | — | — | GPS 同步时间戳；为零时节点用 `ros::Time::now()` |
| `Week` | `uint16` | — | — | GPS 周 |
| `Time` | `float32` | s | — | GPS 周内秒 |
| `Heading` | `float32` | deg | NED | 航向角，正北=0°，顺时针为正 |
| `Pitch` | `float32` | deg | FRD | 俯仰角，抬头为正 |
| `Roll` | `float32` | deg | FRD | 横滚角，右倾为正 |
| `gyro_x/y/z` | `float32` | rad/s | FRD | 三轴角速度（设备输出 deg/s，消息中已转 rad/s） |
| `acc_x/y/z` | `float32` | m/s² | FRD | 三轴加速度（设备输出 g，消息中已转 m/s²） |
| `Lat` | `float64` | deg | WGS84 | 纬度，仅 Status=2 有效 |
| `Lon` | `float64` | deg | WGS84 | 经度，仅 Status=2 有效 |
| `Altitude` | `float32` | m | WGS84 | 高度 |
| `Ve` | `float32` | m/s | NED | 东向速度 |
| `Vn` | `float32` | m/s | NED | 北向速度 |
| `Vd` | `float32` | m/s | NED | 地向速度（向下为正） |
| `Status` | `uint8` | — | — | 0=未初始化, 1=姿态初始化, 2=组合导航正常 |
| `NSV1` | `uint8` | — | — | 主天线卫星数 |
| `NSV2` | `uint8` | — | — | 副天线卫星数 |
| `Age` | `uint8` | 0.1 s | — | 差分龄期，实际秒数 = Age × 0.1 |
#### 加速度单位链（P0#3 修复后）

```
INS 硬件 (g-units)
  → vehicle_interface_ros 驱动 (×9.79 → m/s²)
    → HUAT_InsP2 消息 (m/s²)
      → location.cpp ToCore() 直传 (m/s²)
        → location_mapper: 直接使用，z 轴去重力
        → imu_state_estimator: 直接使用，去重力投影
```

**注意**: 当前驱动 (`vehicle_interface_ros/src/Node.cpp`) 尚未发布 acc 字段（恒为 0.0），
待驱动修复后加速度链路自动生效。

#### INS 质量门控参数

| 参数 | YAML 键 | 默认值 | 单位 | 说明 |
|------|---------|--------|------|------|
| 最低 INS 状态 | `ins_quality/min_ins_status` | 2 | — | Status < 此值时丢弃 |
| 最低卫星数 | `ins_quality/min_satellite_count` | 8 | — | max(NSV1, NSV2) < 此值时丢弃 |
| 最大差分龄期 | `ins_quality/max_diff_age` | 30 | 0.1s | Age > 此值时丢弃（30 = 3.0 秒） |

Mapper 和 Factor Graph 后端统一使用 `max(NSV1, NSV2)` 判断卫星数。

---

### 2.2 锥桶检测输入 — `HUAT_ConeDetections`

| 属性 | 值 |
|------|-----|
| **话题** | `perception/lidar_cluster/detections` |
| **消息类型** | `autodrive_msgs/HUAT_ConeDetections` |
| **发布频率** | ~10 Hz（与 LiDAR 帧率同步） |
| **队列长度** | 10 |

#### 字段契约

| 字段 | 类型 | 单位 | 坐标系 | 说明 |
|------|------|------|--------|------|
| `header.stamp` | `time` | — | — | 继承自输入 PointCloud2 的时间戳 |
| `header.frame_id` | `string` | — | — | LiDAR 坐标系 |
| `points[]` | `Point32` | m | LiDAR | 锥桶质心坐标 (x, y, z) |
| `maxPoints[]` | `Point32` | m | LiDAR | 包围盒最大角 |
| `minPoints[]` | `Point32` | m | LiDAR | 包围盒最小角 |
| `confidence[]` | `float32` | [0, 1] | — | 检测置信度 |
| `obj_dist[]` | `float32` | m | — | 到传感器距离 |
| `color_types[]` | `uint8` | — | — | 0=BLUE, 1=YELLOW, 2=ORANGE_S, 3=ORANGE_B, 4=NONE |

#### 时间戳对齐（P0#2 修复后）

定位节点内部维护 INS 状态环形缓冲区（200 帧 ≈ 2s），收到锥桶检测时
用 `header.stamp` 二分查找并线性插值 INS 状态，消除 INS-LiDAR 时间偏差。

---

## 3. 输出接口

### 3.1 车辆状态 — `HUAT_CarState`

| 属性 | 值 |
|------|-----|
| **话题** | `localization/car_state` |
| **消息类型** | `autodrive_msgs/HUAT_CarState` |
| **频率** | ~100 Hz（跟随 INS 回调） |
| **frame_id** | `world` |
#### 字段契约

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `car_state.x` | `float64` | m | ENU 东向位置（初始点为原点） |
| `car_state.y` | `float64` | m | ENU 北向位置 |
| `car_state.theta` | `float64` | rad | 航向角，初始方向=0，逆时针为正 |
| `car_state_front.x/y/z` | `float64` | m | 前轴中心全局坐标 |
| `car_state_rear.x/y/z` | `float64` | m | 后轴中心全局坐标 |
| `V` | `float32` | m/s | 纵向速度（3D 合速度） |
| `W` | `float32` | rad/s | 偏航角速度 (deprecated → 用 `Wz`) |
| `A` | `float32` | m/s² | 加速度幅值 (deprecated → 用 `Ax`) |
| `Vy` | `float32` | m/s | 横向速度（车体系） |
| `Wz` | `float32` | rad/s | 偏航角速度 |
| `Ax` | `float32` | m/s² | 纵向加速度（车体系） |
| `Ay` | `float32` | m/s² | 横向加速度（车体系） |

#### 坐标系约定

- **原点**: 首次收到有效 INS 数据时的 WGS84 位置
- **X 轴**: 初始航向方向（不是正东）
- **Y 轴**: 初始航向左侧 90°
- **theta = 0**: 初始航向方向
- **theta 正方向**: 逆时针（左转）

### 3.2 锥桶地图 — `HUAT_ConeMap`

| 属性 | 值 |
|------|-----|
| **话题** | `localization/cone_map` |
| **消息类型** | `autodrive_msgs/HUAT_ConeMap` |
| **频率** | ~10 Hz（跟随锥桶检测回调） |
| **frame_id** | `world` |

#### 字段契约 (`HUAT_Cone` 数组)

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `id` | `uint32` | — | 全局唯一 ID，生命周期内不变 |
| `position_global.x/y/z` | `float32` | m | 全局坐标（world 系） |
| `position_baseLink.x/y/z` | `float32` | m | 车体坐标（base_link 系） |
| `confidence` | `uint32` | [0, 1000] | 置信度 × 1000，含观测次数加成 |
| `type` | `uint32` | — | 0=BLUE, 1=YELLOW, 2=ORANGE_S, 3=ORANGE_B, 4=NONE |

**输出范围**: 仅输出距车辆 `local_cone_range` 内的锥桶，经近邻去重后发布。

### 3.3 全局点云地图 — `PointCloud2`

| 属性 | 值 |
|------|-----|
| **话题** | `localization/global_map` |
| **消息类型** | `sensor_msgs/PointCloud2` |
| **频率** | ~10 Hz |
| **frame_id** | `world` |

全部已建图锥桶的 XYZ 点云，用于 RViz 可视化。

### 3.4 位姿 — `PoseStamped`

| 属性 | 值 |
|------|-----|
| **话题** | `localization/pose` |
| **消息类型** | `geometry_msgs/PoseStamped` |
| **频率** | ~100 Hz |
| **frame_id** | `world` |

`position.x/y` = 车辆位置 (m)，`orientation` = 航向四元数（仅 yaw 分量）。

### 3.5 里程计 — `Odometry`

| 属性 | 值 |
|------|-----|
| **话题** | `localization/odom` |
| **消息类型** | `nav_msgs/Odometry` |
| **频率** | ~100 Hz |
| **frame_id** | `world` |
| **child_frame_id** | `base_link` |

- `pose.pose` = 与 PoseStamped 一致
- `twist.twist.linear.x` = 前向速度 (m/s)，由相邻帧差分计算
- `twist.twist.angular.z` = 偏航角速度 (rad/s)，由相邻帧差分计算

### 3.6 Debug 与服务接口（新增）

| 类别 | 名称 | 类型 | 说明 |
|------|------|------|------|
| Debug Topic | `localization/debug/inlier_map` | `HUAT_ConeMap` | 当前帧通过关联/入图的锥桶（inlier） |
| Debug Topic | `localization/debug/outlier_map` | `HUAT_ConeMap` | 当前帧被过滤/拒绝的锥桶（outlier） |
| Debug Topic | `localization/debug/local_map` | `HUAT_ConeMap` | 当前局部地图快照 |
| Debug Topic | `localization/debug/match_pairs` | `std_msgs/String` | 关联对摘要（id/距离/观测坐标） |
| Debug Topic | `localization/debug/relocalization` | `std_msgs/String` | 重定位候选评分摘要（当前为匹配率代理） |
| Service | `localization/map/save` | `std_srvs/Trigger` | 将当前 map 持久化到 `map_persistence/save_path` |
| Service | `localization/map/load` | `std_srvs/Trigger` | 从 `map_persistence/save_path` 载入 map |

---

## 4. TF 契约

### 4.1 发布的变换

| 父坐标系 | 子坐标系 | 发布者 | 频率 |
|----------|----------|--------|------|
| `world` | `base_link` | `location_node` | ~100 Hz |

- **world**: 以首次 INS 有效位置为原点，初始航向为 X 轴的局部 ENU 坐标系
- **base_link**: 车辆中心（IMU 位置）

### 4.2 依赖的变换

定位节点本身不依赖外部 TF。锥桶检测坐标在 LiDAR 系下，
通过 `lidarToIMUDist` 参数硬编码偏移转换到 base_link，再旋转到 world。

### 4.3 质量诊断契约（P0补齐）

定位质量通过 `localization/diagnostics` 的 `localization_entry_health` 暴露，
`HUAT_CarState` 消息本身仍不携带协方差字段（`car_state_covariance_available=false`）。

| 诊断键 | 含义 |
|--------|------|
| `car_state_quality_level` | 2=GOOD, 1=DEGRADED, 0=POOR |
| `car_state_quality_label` | `GOOD` / `DEGRADED` / `POOR` |
| `mapper_state` | `TRACKING` / `DEGRADED` / `INS_ONLY` |
| `cone_last_match_ratio` | 最近一帧局部输出锥桶数/输入锥桶数 |
| `cone_drop_count` | 因失败/INS-only 被丢弃或跳过的锥桶帧累计数 |
| `tf_lag_sec` | 当前 TF 发布时延 (`now - stamp`) |
| `tf_delay_exceed_count` | TF 时延超阈值累计计数 |
| `tf_future_stamp_count` | 未来时间戳累计计数 |
| `tf_stamp_regression_count` | 时间戳回退累计计数 |
| `tf_gap_exceed_count` | 相邻 TF 间隔超阈值累计计数 |
| `mapper_mean_match_distance` | 关联残差均值（m） |
| `mapper_bbox_reject_count` | bbox 过滤拒绝计数 |
| `mapper_geometry_reject_count` | 几何过滤拒绝计数 |
| `mapper_conf_add_reject_count` | 新增锥桶置信度拒绝计数 |
| `mapper_conf_merge_reject_count` | 合并锥桶置信度拒绝计数 |
| `mapper_frozen_reject_count` | map 冻结导致拒绝计数 |
| `map_frozen` | 地图是否处于冻结态 |
| `drift_event_count` | 漂移恢复触发累计计数 |
| `map_save_count` / `map_load_count` | map 保存/加载成功次数 |
| `map_version_tag` | map 版本标签 |

---

## 5. 参数配置层次

```
location_common.yaml          ← 所有赛事共享（话题、坐标系、车体尺寸、INS 质量门控、FG 参数）
  ↓ 覆盖
location_$(mode).yaml          ← 赛事专用（map/mode、合并距离、几何约束）
  ↓ 覆盖
vehicle/$(vehicle)/localization.yaml  ← 车辆专用（车体尺寸覆盖）
  ↓ 覆盖
extra_config (mission overlay)  ← 可选任务级覆盖
```

### 5.1 赛事模式参数表

| 参数 | accel | skidpad | track | 说明 |
|------|-------|---------|-------|------|
| `map/mode` | `accel` | `skidpad` | `track` | 几何过滤模式 |
| `map/merge_distance` | 1.5 | 2.0 | 2.5 | 锥桶合并距离 (m) |
| `map/max_map_size` | 60 | 100 | 500 | 最大锥桶数 |
| `map/local_cone_range` | 30.0 | 35.0 | 30.0 | 局部输出范围 (m) |
| `map/min_confidence_to_add` | 0.35 | 0.2 | 0.3 | 新锥桶入图门槛 |
| `map/min_confidence_to_merge` | 0.15 | 0.1 | 0.15 | 合并门槛 |
| `map/track_width` | 3.0 | 3.0 | 3.5 | 赛道宽度 (m) |
| `map/cone_y_max` | 2.5 | — | — | Y 轴过滤 (仅 accel) |
| `map/enable_circle_validation` | — | true | — | 圆弧验证 (仅 skidpad) |
| `map/circle_radius` | — | 15.25 | — | 圆半径 (m) |
| `map/circle_center_dist` | — | 18.25 | — | 两圆心距 (m) |
| `map/circle_tolerance` | — | 2.0 | — | 圆弧容差 (m) |
| `map/freeze/enabled` | false | false | false | 第一圈建图后冻结 |
| `map/freeze/freeze_after_frames` | 300 | 300 | 300 | 冻结触发帧数 |
| `map/freeze/freeze_after_cones` | 120 | 120 | 120 | 冻结触发锥桶数 |

---

## 6. 已知限制与待修复项

| 编号 | 问题 | 状态 | 影响 |
|------|------|------|------|
| P0#1 | map_mode 未传播到 YAML | **已修复** | 几何过滤现已按赛事生效 |
| P0#2 | INS-LiDAR 时间戳未对齐 | **已修复** | 新增 INS 缓冲区 + 插值 |
| P0#3 | 加速度双重缩放 | **已修复** | 移除 ×9.79，直接使用 m/s² |
| P0#4 | GNSS 门控参数硬编码 | **已修复** | 暴露为 `ins_quality/*` ROS 参数 |
| — | 驱动未发布 acc/Roll/NSV/Age | **待修复** | `vehicle_interface_ros` 需补全字段 |
| — | FG 后端仅 shadow mode | 设计中 | M2 里程碑升级为主后端 |

---

## 7. 变更控制

对本文档中冻结的接口做 breaking change 时，需要：

1. 在 `autodrive_msgs/` 中新增 V2 消息（不修改原消息）
2. 双发布过渡期 ≥ 1 周
3. 下游模块（planning_ros、control_ros、fsd_visualization）确认切换后，废弃旧消息
