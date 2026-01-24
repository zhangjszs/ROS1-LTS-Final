# race_rviz_viz

用于 FSAC 比赛的 RViz 可视化 ROS 包，支持锥桶（可选）、车辆车身/四轮和路径在全局坐标系下的可视化。

---

## 功能概述

| 功能 | 订阅话题 | 消息类型 | 发布话题 | 说明 |
|------|----------|----------|----------|------|
| 锥桶可视化（可选） | `/coneMap` | `autodrive_msgs/HUAT_ConeMap` | `/coneMarker` | Marker (CYLINDER) |
| 车辆可视化 | `/Carstate` | `autodrive_msgs/HUAT_CarState` | `/carBody` `/whole` | CUBE 车身 + 四轮 |
| 路径可视化 | `/path_global` | `nav_msgs/Path` | `/viz/path` | 直接转发，frame_id=global_frame |
| High Speed Tracking 可视化（可选） | `/AS/P/high_speed_tracking/viz` | `autodrive_msgs/HUAT_HighSpeedViz` | `/viz/high_speed_tracking/markers/*` | 基于 HUAT_HighSpeedViz 绘制 |
| PathLimits 可视化（可选） | `/AS/P/pathlimits/partial` | `autodrive_msgs/HUAT_PathLimits` | `/viz/pathlimits/path` `/viz/pathlimits/left` `/viz/pathlimits/right` | 基于 PathLimits 绘制 |
| Lidar Cluster BoundingBox 可视化（可选） | `/cone_position` | `autodrive_msgs/HUAT_ConeDetections` | `/viz/lidar_cluster/bounding_box` | 基于 min/max 生成包围框 |
| TF 广播 | `/Carstate` | `autodrive_msgs/HUAT_CarState` | TF: `global_frame` → `vehicle_frame` | 可配置 |

---

## 编译

### catkin_make

```bash
cd ~/2025huat
catkin_make --only-pkg-with-deps race_rviz_viz
source devel/setup.bash
```

---

## 运行

### 基本运行

```bash
roslaunch race_rviz_viz viz.launch
```

### 自定义参数运行

```bash
roslaunch race_rviz_viz viz.launch \
  global_frame:=velodyne \
  cones_topic:=/coneMap \
  carstate_topic:=/Carstate \
  publish_high_speed_tracking_viz:=true \
  high_speed_tracking_viz_topic:=/AS/P/high_speed_tracking/viz \
  cone_radius:=0.15 \
  wheel_half_track:=0.35
```

---

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `global_frame` | string | `velodyne` | 全局坐标系名称 |
| `vehicle_frame` | string | `base_link` | 车辆坐标系名称（TF 子坐标系） |
| `publish_tf` | bool | `true` | 是否广播 TF |
| `publish_cones` | bool | `false` | 是否发布锥桶 Marker |
| `cones_topic` | string | `/coneMap` | 锥桶地图输入话题 |
| `carstate_topic` | string | `/Carstate` | 车辆位姿输入话题 |
| `path_topic` | string | `/path_global` | 路径输入话题 |
| `publish_high_speed_tracking_viz` | bool | `false` | 是否开启 High Speed Tracking 可视化 |
| `high_speed_tracking_viz_topic` | string | `/AS/P/high_speed_tracking/viz` | High Speed Tracking 可视化输入（HUAT_HighSpeedViz） |
| `publish_pathlimits` | bool | `false` | 是否发布 PathLimits 可视化 |
| `pathlimits_topic` | string | `/AS/P/pathlimits/partial` | PathLimits 输入话题 |
| `pathlimits_path_topic` | string | `/viz/pathlimits/path` | PathLimits 路径输出 |
| `pathlimits_left_topic` | string | `/viz/pathlimits/left` | PathLimits 左侧锥桶输出 |
| `pathlimits_right_topic` | string | `/viz/pathlimits/right` | PathLimits 右侧锥桶输出 |
| `pathlimits_line_width` | double | `0.2` | PathLimits 路径线宽 |
| `pathlimits_path_color_r/g/b/a` | double | `0.0/1.0/0.0/1.0` | PathLimits 路径颜色 |
| `pathlimits_left_color_r/g/b/a` | double | `0.0/0.4/1.0/1.0` | PathLimits 左侧锥桶颜色 |
| `pathlimits_right_color_r/g/b/a` | double | `1.0/1.0/0.0/1.0` | PathLimits 右侧锥桶颜色 |
| `publish_lidar_cluster_bboxes` | bool | `true` | 是否发布 Lidar Cluster 包围框 |
| `lidar_cluster_cone_topic` | string | `/cone_position` | Lidar Cluster 锥桶输入 |
| `cone_marker_topic` | string | `/coneMarker` | 锥桶 Marker 输出话题 |
| `car_body_topic` | string | `/carBody` | 车身 Marker 输出话题 |
| `wheels_topic` | string | `/whole` | 车轮 Marker 输出话题 |
| `path_out_topic` | string | `/viz/path` | 路径转发输出话题 |
| `high_speed_tracking_triangulation_out_topic` | string | `/viz/high_speed_tracking/markers/triangulation` | High Speed Tracking 三角剖分 MarkerArray 输出 |
| `high_speed_tracking_midpoints_out_topic` | string | `/viz/high_speed_tracking/markers/midpoints` | High Speed Tracking 中点 MarkerArray 输出 |
| `high_speed_tracking_way_out_topic` | string | `/viz/high_speed_tracking/markers/way` | High Speed Tracking 路径 MarkerArray 输出 |
| `lidar_cluster_bbox_out_topic` | string | `/viz/lidar_cluster/bounding_box` | Lidar Cluster BoundingBox 输出 |
| `cone_radius` | double | `0.15` | 锥桶半径（米） |
| `cone_height` | double | `0.35` | 锥桶高度（米） |
| `cone_color_r/g/b/a` | double | `1.0/0.5/0.0/1.0` | 锥桶颜色 |
| `lidar_to_imu` | double | `1.87` | 激光雷达到 IMU 的前向偏移（米） |
| `car_body_front` | double | `0.0` | 车身前沿相对 IMU 的 x 偏移 |
| `car_body_rear` | double | `-1.5` | 车身后沿相对 IMU 的 x 偏移 |
| `car_body_half_width` | double | `0.25` | 车身半宽 |
| `car_body_line_width` | double | `0.3` | 车身线宽（保留参数） |
| `car_body_height` | double | `0.5` | 车身高度 |
| `wheel_front_x` | double | `-0.2` | 前轮中心相对 IMU 的 x 偏移 |
| `wheel_rear_x` | double | `-1.2` | 后轮中心相对 IMU 的 x 偏移 |
| `wheel_half_track` | double | `0.35` | 轮距一半 |
| `wheel_diameter` | double | `0.5` | 轮胎直径 |
| `wheel_width` | double | `0.10` | 轮胎厚度 |
| `wheel_color_r/g/b/a` | double | `0.0/1.0/0.0/1.0` | 轮胎颜色 |

---

## RViz 配置步骤

1. Fixed Frame 设置为 `global_frame`（默认 `velodyne`）。
2. 添加 `/coneMarker` → `Marker`（如开启 publish_cones）。
3. 添加 `/carBody` → `Marker`。
4. 添加 `/whole` → `Marker`。
5. 如需路径显示，添加 `/viz/path` → `Path`。
6. 如需 High Speed Tracking 可视化，添加 `/viz/high_speed_tracking/markers/triangulation` `/viz/high_speed_tracking/markers/midpoints` `/viz/high_speed_tracking/markers/way` → `MarkerArray`。
7. 如需 PathLimits 可视化，添加 `/viz/pathlimits/path` `/viz/pathlimits/left` `/viz/pathlimits/right` → `Marker`。
8. 如需 Lidar Cluster 可视化，添加 `/viz/lidar_cluster/bounding_box` → `MarkerArray`。

---

## License

MIT License
