# Localization ROS Topics

本文档描述 localization_ros 包中各节点的订阅/发布话题与意义，并基于 ros1-structure-guide 做了命名整理。

## 总体约定
- 话题使用小写 + 下划线风格，参数统一在 `config/*.yaml` 内配置。
- 话题名使用相对名（无前导 `/`），方便后续通过 launch 或 namespace 重定向。
- 若外部传感器仍发布 `/pbox_pub/Ins`，请在 `location.yaml` / `state_estimator.yaml` 中将 `topics/ins` 改为 `/pbox_pub/Ins`。

## 节点清单

### localization_ros/location_node
订阅：
- `topics/ins` (`autodrive_msgs/HUAT_Asensing`)：INS 原始数据，用于更新位姿（仅在 `use_external_carstate=false` 时订阅）
- `topics/car_state_in` (`autodrive_msgs/HUAT_CarState`)：外部 car_state（仅在 `use_external_carstate=true` 时订阅）
- `topics/cone` (`autodrive_msgs/HUAT_ConeDetections`)：锥桶检测结果

发布：
- `topics/car_state_out` (`autodrive_msgs/HUAT_CarState`)：融合后的车辆状态
- `topics/cone_map` (`autodrive_msgs/HUAT_ConeMap`)：锥桶地图
- `topics/global_map` (`sensor_msgs/PointCloud2`)：全局点云地图（仅在更新成功时）
- `topics/pose` (`geometry_msgs/PoseStamped`)：车辆位姿（由 car_state 转换）
- `topics/odom` (`nav_msgs/Odometry`)：里程计输出（由 car_state 转换）
- `tf`：发布 `frames/world` -> `frames/base_link` 变换

默认话题（`config/location.yaml`）：
- ins: `sensors/ins`
- car_state_in: `localization/car_state`
- cone: `perception/lidar_cluster/detections`
- car_state_out: `localization/car_state`
- cone_map: `localization/cone_map`
- global_map: `localization/global_map`
- pose: `localization/pose`
- odom: `localization/odom`
- frames/world: `world`
- frames/base_link: `base_link`

### localization_ros/state_estimator_node
订阅：
- `topics/ins` (`autodrive_msgs/HUAT_Asensing`)：INS 原始数据

发布：
- `topics/car_state` (`autodrive_msgs/HUAT_CarState`)：由 ESKF 估计的车辆状态

默认话题（`config/state_estimator.yaml`）：
- ins: `sensors/ins`
- car_state: `localization/car_state`

## 话题改名/弃用
- `/Carstate` -> `localization/car_state`
- `/coneMap` -> `localization/cone_map`
- `/globalMapOnly` -> `localization/global_map`
- `/pbox_pub/Ins` -> `sensors/ins`（推荐内部统一名）

## 下游需要同步更新
- `fsd_visualization` 的 `cone_visualizer`：`/coneMap` -> `localization/cone_map`
- `planning_ros` 的高速跟踪配置：`/coneMap` -> `localization/cone_map`

## 备注
- `pose/odom/tf` 使用 `car_state` 的 `x/y/theta` 生成，`z` 目前固定为 0。

## 建议删除/减少
- 旧的驼峰/大写话题名：`/Carstate`、`/coneMap`、`/globalMapOnly`
- 若 `global_map` 很少使用，可考虑改为按需发布或关闭
