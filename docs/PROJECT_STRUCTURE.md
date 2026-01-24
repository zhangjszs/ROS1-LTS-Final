# 2025HUAT FSD 项目结构文档

## 目录结构

```
2025huat/
├── src/                          # 源代码目录
│   ├── autodrive_msgs/           # 统一消息定义
│   │
│   ├── perception_core/          # 感知算法核心 (无 ROS 依赖)
│   ├── perception_ros/           # 感知 ROS 包装层
│   │
│   ├── planning_core/            # 规划算法核心 (无 ROS 依赖)
│   ├── planning_ros/             # 规划 ROS 包装层
│   │
│   ├── control_core/             # 控制算法核心 (无 ROS 依赖)
│   ├── control_ros/              # 控制 ROS 包装层
│   │
│   ├── localization_core/        # 定位算法核心 (无 ROS 依赖)
│   ├── localization_ros/         # 定位 ROS 包装层
│   │
│   ├── vehicle_interface_core/   # 车辆接口核心
│   ├── vehicle_interface_ros/    # 车辆接口 ROS 层
│   ├── ros_vehicle_interface/    # 车辆 UDP 通信
│   │
│   ├── vehicle_racing_num_core/  # 比赛编号核心
│   ├── vehicle_racing_num_ros/   # 比赛编号 ROS 层
│   ├── ros_vehicle_racing_num/   # 比赛编号节点
│   │
│   ├── ins/                      # INS 消息兼容桥接
│   ├── fsd_visualization/        # 统一可视化节点
│   ├── fsd_launch/               # 统一启动配置
│   │
│   └── _deprecated/              # 废弃代码 (已忽略)
│
├── autoStartGkj/                 # 实车启动脚本
├── devel/                        # catkin 构建输出
├── build/                        # 构建中间文件
├── logs/                         # 构建日志
└── docs/                         # 文档目录
```

## 启动文件结构

```
src/fsd_launch/launch/
├── missions/                     # 任务模式
│   ├── trackdrive.launch         # 高速循迹
│   ├── skidpad.launch            # 8字绕环
│   ├── acceleration.launch       # 直线加速
│   └── autocross.launch          # 综合赛道
│
├── subsystems/                   # 子系统
│   ├── perception.launch         # 感知 (点云聚类)
│   ├── planning.launch           # 规划 (路径规划)
│   ├── control.launch            # 控制 (路径跟踪)
│   ├── localization.launch       # 定位 (状态估计)
│   └── vehicle.launch            # 车辆接口
│
└── tools/                        # 工具
    ├── rviz.launch               # RViz 可视化
    ├── rosbag_play.launch        # Rosbag 回放
    ├── debug.launch              # 调试工具
    └── topic_bridge.launch       # 话题桥接
```

## RViz 配置文件

```
src/fsd_visualization/rviz/
├── fsd_main.rviz                 # 综合主视图 (推荐，可自行修改保存)
├── global_viz.rviz               # 全局俯视图
├── pointcloud_only.rviz          # 仅点云视图
└── fsd_default.rviz              # 默认配置
```

---

## 启动命令

### 仿真模式 (Rosbag 回放)

```bash
# 高速循迹 - 综合视图 (默认)
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag

# 高速循迹 - 指定 RViz 模式
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=main
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=dual
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=global
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=pointcloud

# 8字绕环
roslaunch fsd_launch skidpad.launch simulation:=true bag:=/path/to/bag.bag

# 直线加速
roslaunch fsd_launch acceleration.launch simulation:=true bag:=/path/to/bag.bag

# 综合赛道
roslaunch fsd_launch autocross.launch simulation:=true bag:=/path/to/bag.bag
```

### 实车模式

```bash
# 方式1: 使用启动脚本
bash autoStartGkj/start.sh

# 方式2: 直接启动
roslaunch fsd_launch trackdrive.launch
```

### 单独启动子系统

```bash
# 感知
roslaunch fsd_launch perception.launch

# 规划
roslaunch fsd_launch planning.launch planner:=high_speed

# 控制
roslaunch fsd_launch control.launch

# 定位
roslaunch fsd_launch localization.launch

# 可视化
roslaunch fsd_launch rviz.launch mode:=main
```

### RViz 模式说明

| 模式 | 参数 | 说明 |
|------|------|------|
| 综合主视图 | `rviz_mode:=main` | 包含所有可视化元素，可自行修改保存 |
| 全局俯视图 | `rviz_mode:=global` | 俯视角度，适合查看全局路径 |
| 仅点云 | `rviz_mode:=pointcloud` | 仅显示点云，用于调试感知 |
| 双窗口 | `rviz_mode:=dual` | 同时显示点云和全局视图 |

---

## 主要话题

### 感知 (Perception)
| 话题 | 类型 | 描述 |
|------|------|------|
| `/velodyne_points` | sensor_msgs/PointCloud2 | 原始点云 |
| `/points_ground` | sensor_msgs/PointCloud2 | 地面点云 |
| `/points_no_ground` | sensor_msgs/PointCloud2 | 非地面点云 |
| `/coneMap` | autodrive_msgs/HUAT_map | 锥桶地图 |
| `/coneMarker` | visualization_msgs/Marker | 锥桶标记 |

### 定位 (Localization)
| 话题 | 类型 | 描述 |
|------|------|------|
| `/pbox_pub/Ins` | autodrive_msgs/HUAT_Asensing | INS 数据 |
| `/Carstate` | autodrive_msgs/HUAT_Carstate | 车辆状态 |

### 规划 (Planning)
| 话题 | 类型 | 描述 |
|------|------|------|
| `/path_global` | nav_msgs/Path | 全局路径 |
| `/viz/high_speed_tracking/markers/*` | MarkerArray | 规划可视化 |

### 控制 (Control)
| 话题 | 类型 | 描述 |
|------|------|------|
| `/vehcileCMDMsg` | autodrive_msgs/vehicle_cmd | 控制指令 |

### 可视化 (Visualization)
| 话题 | 类型 | 描述 |
|------|------|------|
| `/fsd/viz/cones` | MarkerArray | 锥桶可视化 |
| `/fsd/viz/path` | MarkerArray | 路径可视化 |
| `/fsd/viz/boundaries` | MarkerArray | 边界可视化 |
| `/fsd/viz/vehicle` | MarkerArray | 车辆可视化 |

---

## 构建命令

```bash
# 构建所有包
cd ~/2025huat
catkin build

# 构建指定包
catkin build perception_ros

# 清理构建
catkin clean

# 重新构建
catkin clean -y && catkin build
```

## 坐标系

| Frame ID | 描述 |
|----------|------|
| `velodyne` | 激光雷达坐标系 (Fixed Frame) |
| `base_link` | 车辆基座坐标系 |

---

## 技术规范

- **ROS 版本:** Noetic
- **C++ 标准:** C++17
- **构建工具:** catkin_tools
- **架构模式:** Core + ROS Wrapper 分层
