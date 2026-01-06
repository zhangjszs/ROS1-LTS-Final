# race_rviz_viz

用于 FSAC 比赛的 RViz 可视化 ROS 包，支持锥桶（可选）、车辆车身/四轮和路径在全局坐标系下的可视化。

---

## 功能概述

| 功能 | 订阅话题 | 消息类型 | 发布话题 | 说明 |
|------|----------|----------|----------|------|
| 锥桶可视化（可选） | `/coneMap` | `common_msgs/HUAT_map` | `/coneMarker` | Marker (CYLINDER) |
| 车辆可视化 | `/Carstate` | `common_msgs/HUAT_Carstate` | `/carBody` `/whole` | CUBE 车身 + 四轮 |
| 路径可视化 | `/path_global` | `nav_msgs/Path` | `/viz/path` | 直接转发，frame_id=global_frame |
| Urinay MarkerArray 可视化（可选） | `/AS/P/urinay/markers/*` | `visualization_msgs/MarkerArray` | `/viz/urinay/markers/*` | 原样转发 |
| TF 广播 | `/Carstate` | `common_msgs/HUAT_Carstate` | TF: `global_frame` → `vehicle_frame` | 可配置 |

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
  publish_urinay_markers:=true \
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
| `publish_urinay_markers` | bool | `false` | 是否转发 Urinay MarkerArray |
| `urinay_triangulation_topic` | string | `/AS/P/urinay/markers/triangulation` | Urinay 三角剖分 MarkerArray 输入 |
| `urinay_midpoints_topic` | string | `/AS/P/urinay/markers/midpoints` | Urinay 中点 MarkerArray 输入 |
| `urinay_way_topic` | string | `/AS/P/urinay/markers/way` | Urinay 路径 MarkerArray 输入 |
| `cone_marker_topic` | string | `/coneMarker` | 锥桶 Marker 输出话题 |
| `car_body_topic` | string | `/carBody` | 车身 Marker 输出话题 |
| `wheels_topic` | string | `/whole` | 车轮 Marker 输出话题 |
| `path_out_topic` | string | `/viz/path` | 路径转发输出话题 |
| `urinay_triangulation_out_topic` | string | `/viz/urinay/markers/triangulation` | Urinay 三角剖分 MarkerArray 输出 |
| `urinay_midpoints_out_topic` | string | `/viz/urinay/markers/midpoints` | Urinay 中点 MarkerArray 输出 |
| `urinay_way_out_topic` | string | `/viz/urinay/markers/way` | Urinay 路径 MarkerArray 输出 |
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
6. 如需 Urinay 可视化，添加 `/viz/urinay/markers/triangulation` `/viz/urinay/markers/midpoints` `/viz/urinay/markers/way` → `MarkerArray`。

---

## License

MIT License
