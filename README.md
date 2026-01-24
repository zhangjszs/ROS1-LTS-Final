# 2025HUAT - FSD 无人驾驶系统

## 项目架构

本项目采用 **Core + ROS Wrapper** 分层架构，实现 ROS 与算法逻辑的解耦。

```
src/
├── autodrive_msgs/       # 统一消息定义
├── fsd_launch/           # 统一启动配置
│   └── launch/
│       ├── missions/     # 任务模式: trackdrive, skidpad, acceleration, autocross
│       ├── subsystems/   # 子系统: perception, planning, control, localization
│       └── tools/        # 工具: rviz, rosbag_play, debug
├── fsd_visualization/    # 统一可视化节点
├── perception_core/      # 感知算法核心 (无 ROS 依赖)
├── perception_ros/       # 感知 ROS 包装层
├── planning_core/        # 规划算法核心 (无 ROS 依赖)
├── planning_ros/         # 规划 ROS 包装层
├── control_core/         # 控制算法核心 (无 ROS 依赖)
├── control_ros/          # 控制 ROS 包装层
├── localization_core/    # 定位算法核心 (无 ROS 依赖)
├── localization_ros/     # 定位 ROS 包装层
├── ros_vehicle_interface/# 车辆通信接口
├── ros_vehicle_racing_num/# 比赛编号管理
└── ins/                  # INS 消息兼容桥接
```

## 快速开始

### 构建
```bash
cd ~/2025huat
catkin build
source devel/setup.bash
```

### 运行

**仿真模式（rosbag 回放）:**
```bash
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag
```

**实车模式:**
```bash
bash autoStartGkj/start.sh
```

### RViz 可视化模式
```bash
# 双窗口模式 (默认)
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=dual

# 仅点云
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=pointcloud

# 仅全局俯视图
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=global
```

## 任务模式

| 任务 | 启动命令 | 说明 |
|------|----------|------|
| TrackDrive | `roslaunch fsd_launch trackdrive.launch` | 高速循迹 |
| Skidpad | `roslaunch fsd_launch skidpad.launch` | 8字绕环 |
| Acceleration | `roslaunch fsd_launch acceleration.launch` | 直线加速 |
| Autocross | `roslaunch fsd_launch autocross.launch` | 综合赛道 |

## Launch 配置总览（fsd_launch）

### missions（任务级）
- `src/fsd_launch/launch/missions/trackdrive.launch`
- `src/fsd_launch/launch/missions/skidpad.launch`
- `src/fsd_launch/launch/missions/autocross.launch`
- `src/fsd_launch/launch/missions/acceleration.launch`

### subsystems（子系统级）
- `src/fsd_launch/launch/subsystems/perception.launch`
- `src/fsd_launch/launch/subsystems/localization.launch`
- `src/fsd_launch/launch/subsystems/planning.launch`
- `src/fsd_launch/launch/subsystems/control.launch`
- `src/fsd_launch/launch/subsystems/vehicle.launch`

### tools（工具级）
- `src/fsd_launch/launch/tools/rosbag_play.launch`
- `src/fsd_launch/launch/tools/rviz.launch`
- `src/fsd_launch/launch/tools/debug.launch`
- `src/fsd_launch/launch/tools/topic_bridge.launch`

> 说明：当前 `trackdrive.launch` 实际位于 `missions/` 子目录，推荐使用  
> `roslaunch fsd_launch missions/trackdrive.launch ...`

## TrackDrive 启动参数说明

示例（仿真 + 回放 + 双窗口 RViz + 循环）：
```bash
roslaunch fsd_launch missions/trackdrive.launch \
  simulation:=true \
  bag:=/home/kerwin/rosbag/22910_2.bag \
  rviz_mode:=dual \
  loop:=true
```

参数说明（来自 `missions/trackdrive.launch`）：

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `simulation` | `false` | 是否仿真模式；为 `true` 时启用 `/use_sim_time` 并播放 rosbag |
| `bag` | `""` | rosbag 文件路径（仿真模式必填） |
| `rate` | `1.0` | rosbag 播放速率 |
| `loop` | `false` | rosbag 是否循环播放（传给 `tools/rosbag_play.launch`） |
| `launch_rviz` | `true` | 是否启动 RViz（通过 `tools/rviz.launch`） |
| `launch_viz` | `true` | 是否启动 `fsd_visualization` 可视化节点 |
| `rviz_mode` | `main` | RViz 显示模式：`main`/`global`/`pointcloud`/`dual` |

## 主要话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/velodyne_points` | PointCloud2 | 原始点云 |
| `/coneMap` | HUAT_map | 锥桶地图 |
| `/Carstate` | HUAT_Carstate | 车辆状态 |
| `/path_global` | nav_msgs/Path | 全局路径 |
| `/vehcileCMDMsg` | vehicle_cmd | 控制指令 |
| `/fsd/viz/*` | Marker/MarkerArray | 可视化标记 |

## 技术规范

- **ROS 版本:** Noetic
- **C++ 标准:** C++17
- **构建工具:** catkin_tools
