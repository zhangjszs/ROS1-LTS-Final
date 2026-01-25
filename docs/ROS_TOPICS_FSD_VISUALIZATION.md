# FSD Visualization ROS Topics

本文档描述 `fsd_visualization` 包中可视化节点的订阅/发布话题与意义，并基于 ros1-structure-guide 做了命名整理。

## 总体约定
- 话题使用小写 + 下划线风格，参数统一在节点私有参数 `~` 下配置。
- 话题名使用相对名（无前导 `/`），便于通过 launch 或 namespace 重定向。

## 节点清单

### fsd_visualization/fsd_viz_node
功能：统一启动可视化组件（锥桶/路径/车辆）。
- 通过 `~enable_cones` / `~enable_path` / `~enable_vehicle` 控制子模块启用。
- 话题由各子模块参数决定（见下）。

### fsd_visualization/ConeVisualizer
订阅：
- `topics/cone_detections` (`autodrive_msgs/HUAT_ConeDetections`)：锥桶检测结果（局部坐标）
- `topics/cone_map` (`autodrive_msgs/HUAT_ConeMap`)：锥桶地图（全局坐标）

发布：
- `topics/markers` (`visualization_msgs/MarkerArray`)：锥桶可视化 Marker

默认话题（代码默认）：
- cone_detections: `perception/lidar_cluster/detections`
- cone_map: `localization/cone_map`
- markers: `fsd/viz/cones`

### fsd_visualization/PathVisualizer
订阅：
- `topics/path_partial` (`autodrive_msgs/HUAT_PathLimits`)：高速规划局部路径
- `topics/path_full` (`autodrive_msgs/HUAT_PathLimits`)：高速规划完整路径
- `topics/nav_path` (`nav_msgs/Path`)：直线/滑行等模块输出的通用路径

发布：
- `topics/markers` (`visualization_msgs/MarkerArray`)：路径可视化 Marker
- `topics/boundaries` (`visualization_msgs/MarkerArray`)：赛道边界 Marker

默认话题（代码默认）：
- path_partial: `planning/high_speed_tracking/pathlimits/partial`
- path_full: `planning/high_speed_tracking/pathlimits/full`
- nav_path: `planning/line_detection/path`
- markers: `fsd/viz/path`
- boundaries: `fsd/viz/boundaries`

### fsd_visualization/VehicleVisualizer
订阅：
- `topics/car_state` (`autodrive_msgs/HUAT_CarState`)：车辆状态

发布：
- `topics/markers` (`visualization_msgs/MarkerArray`)：车辆与轨迹 Marker

默认话题（代码默认）：
- car_state: `localization/car_state`
- markers: `fsd/viz/vehicle`

## 话题改名/弃用
- `/Carstate` -> `localization/car_state`
- `/coneMap` -> `localization/cone_map`
- `/AS/P/pathlimits/*` -> `planning/high_speed_tracking/pathlimits/*`
- `/path_global` -> `planning/line_detection/path`

## 建议增加
- 统一入口 `planning/path`（将直线/滑行/高速路径汇总，便于只订阅一个路径话题）
- 车辆姿态可视化支持 `localization/pose`（`geometry_msgs/PoseStamped`），便于与标准工具对齐

## 建议删除/减少
- 旧的绝对话题名（`/Carstate`、`/coneMap`、`/AS/P/...`）
