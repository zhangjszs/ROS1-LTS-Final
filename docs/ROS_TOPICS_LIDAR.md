# LiDAR 点云处理（perception_ros / lidar_cluster）

本文档记录 **LiDAR 点云处理功能包** 的订阅 / 发布话题及其含义。当前仅覆盖激光雷达模块。

## 1) 包与节点

**包**：`perception_ros`  
**核心节点**：`lidar_cluster_node`（nodelet 方式运行，参见 `perception_ros/launch/lidar_cluster.launch`）  
**命名空间**：默认 `perception/lidar_cluster`（可在 launch 里改 `ns`）

> 注意：话题名称默认均为**相对名**，由命名空间统一管理。

## 2) 订阅话题（输入）

| 话题 | 类型 | 说明 |
|---|---|---|
| `points/raw` | `sensor_msgs/PointCloud2` | 原始点云输入。默认在 launch 中映射到 `/velodyne_points` |

参数位置：`perception_ros/config/lidar_cluster_example.yaml`  
参数键：`topics/input`

## 3) 发布话题（输出）

| 话题 | 类型 | 说明 |
|---|---|---|
| `points/passthrough` | `sensor_msgs/PointCloud2` | 直通滤波+体素滤波后的点云 |
| `points/no_ground` | `sensor_msgs/PointCloud2` | 地面分割后的**非地面**点云 |
| `points/cones` | `sensor_msgs/PointCloud2` | 被判定为锥桶的点云聚类 |
| `detections` | `autodrive_msgs/HUAT_ConeDetections` | 锥桶检测结果（中心点、bbox、置信度、距离等） |
| `markers/cones` | `visualization_msgs/MarkerArray` | 锥桶可视化 bbox/距离文字 |
| `markers/all` | `visualization_msgs/MarkerArray` | 所有聚类可视化（当 `vis != 0` 时才发布） |

参数位置：`perception_ros/config/lidar_cluster_example.yaml`  
参数键：`topics/points/*`, `topics/detections`, `topics/markers/*`

## 4) RViz 常用订阅

建议在 RViz 订阅以下话题：
- `/perception/lidar_cluster/points/passthrough`
- `/perception/lidar_cluster/points/no_ground`
- `/perception/lidar_cluster/points/cones`
- `/perception/lidar_cluster/markers/cones`

## 5) 与其他模块的关系（默认）

| 下游模块 | 默认订阅 |
|---|---|
| `planning_ros/line_detection` | `/perception/lidar_cluster/detections` |
| `planning_ros/skidpad_detection` | `/perception/lidar_cluster/detections` |
| `localization_ros/location_node` | `/perception/lidar_cluster/detections` |
| `fsd_visualization/cone_visualizer` | `/perception/lidar_cluster/detections` |

> 若需改名，请通过各自的 `topics/*` 参数或 launch remap 统一配置。

