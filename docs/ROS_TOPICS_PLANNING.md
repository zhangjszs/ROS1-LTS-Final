# Planning ROS Topics

本文档描述 planning_ros 包中各节点的订阅/发布话题与意义，并基于 ros1-structure-guide 做了命名整理。

## 总体约定
- 话题使用小写 + 下划线风格，参数统一在 `config/*.yaml` 内配置。
- 话题名使用相对名（无前导 `/`），便于通过 launch 或 namespace 重定向。

## 节点清单

### planning_ros/line_detection_node
订阅：
- `topics/cone` (`autodrive_msgs/HUAT_ConeDetections`)：锥桶检测结果（用于直线赛道边界拟合）
- `topics/car_state` (`autodrive_msgs/HUAT_CarState`)：车辆状态

发布：
- `topics/path` (`nav_msgs/Path`)：规划出的直线赛道中心线
- `topics/finish` (`std_msgs/Bool`)：终点触发信号

默认话题（`config/line_detection.yaml`）：
- cone: `perception/lidar_cluster/detections`
- car_state: `localization/car_state`
- path: `planning/line_detection/path`
- finish: `planning/line_detection/finish_signal`

### planning_ros/skidpad_detection_node
订阅：
- `topics/cone` (`autodrive_msgs/HUAT_ConeDetections`)：锥桶检测结果
- `topics/car_state` (`autodrive_msgs/HUAT_CarState`)：车辆状态

发布：
- `topics/log_path` (`nav_msgs/Path`)：滑行赛道规划路径（用于控制或调试）
- `topics/approaching_goal` (`std_msgs/Bool`)：接近终点标志

默认话题（`config/skidpad_detection.yaml`）：
- cone: `perception/lidar_cluster/detections`
- car_state: `localization/car_state`
- log_path: `planning/skidpad/log_path`
- approaching_goal: `planning/skidpad/approaching_goal`

### planning_ros/high_speed_tracking_node
订阅：
- `input_cones_topic` (`autodrive_msgs/HUAT_ConeMap`)：锥桶地图
- `input_pose_topic` (`autodrive_msgs/HUAT_CarState`)：车辆状态

发布：
- `output_full_topic` (`autodrive_msgs/HUAT_PathLimits`)：回环后完整路径
- `output_partial_topic` (`autodrive_msgs/HUAT_PathLimits`)：回环前局部路径
- `stop_topic` (`autodrive_msgs/HUAT_Stop`)：停止指令
- `viz_topic` (`autodrive_msgs/HUAT_HighSpeedViz`)：可视化数据（可选）

默认话题（`config/high_speed_tracking.yml`）：
- input_cones_topic: `localization/cone_map`
- input_pose_topic: `localization/car_state`
- output_full_topic: `planning/high_speed_tracking/pathlimits/full`
- output_partial_topic: `planning/high_speed_tracking/pathlimits/partial`
- stop_topic: `planning/high_speed_tracking/stop`
- viz_topic: `planning/high_speed_tracking/viz`

## 话题改名/弃用
- `/Carstate` -> `localization/car_state`
- `/coneMap` -> `localization/cone_map`
- `/line_planned_path` -> `planning/line_detection/path`
- `/line_finish_signal` -> `planning/line_detection/finish_signal`
- `/AS/P/pathlimits/*` -> `planning/high_speed_tracking/pathlimits/*`
- `/stopTheCar` -> `planning/high_speed_tracking/stop`
- `/skidpad_detection`（仅参数占位，代码未使用）建议删除

## 建议增加
- `planning/path`（统一输出路径，便于控制只订阅一个话题）
- `planning/stop`（统一的停止指令，替代各子模块单独 stop 话题）

## 建议删除/减少
- 旧的绝对话题名与 `/AS/P` 命名空间下的路径话题
- 未使用的参数 `filtered_topic_name`
