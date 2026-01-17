# Line Detection - 直线路径规划

## 概述

`line_detection` 包使用霍夫变换（Hough Transform）算法对直线加速赛中的锥桶进行直线检测，并生成相应的路径规划。

## 算法原理

### 霍夫直线检测

霍夫直线检测基于点与线的对偶性：
- 图像空间中的每条直线在参数空间中对应一个点
- 图像空间中同一条直线上的所有点，在参数空间中映射为同一个点

通过将直线方程从笛卡尔坐标系转换为极坐标形式：
```
ρ = x*cos(θ) + y*sin(θ)
```

其中：
- ρ: 从原点到直线的距离
- θ: x轴到法线的角度

### 算法流程

1. **数据获取**: 从感知模块订阅锥桶位置信息（`/cone_position`）
2. **数据过滤**: 根据距离和区域过滤锥桶
3. **霍夫变换**: 将锥桶位置映射到霍夫空间进行直线检测
4. **边界线选择**: 选择两侧边界线（平行线）
5. **中心线计算**: 计算两条边界线的中心线作为行驶路径
6. **路径生成**: 沿中心线生成路径点
7. **坐标转换**: 将路径从车身坐标系转换到世界坐标系（东北天坐标系）

### 坐标系转换

使用公式（5-31）将路径从车身坐标系转换到世界坐标系：
```
X = (x + 1.87) * sin(θ)
Y = (y + 1.87) * cos(θ)
```

其中 1.87m 是惯导与车身坐标系原点的相对距离，θ 是车辆偏航角。

## 功能特性

- 订阅 `/cone_position` 话题获取锥桶位置
- 订阅 `/Carstate` 话题获取车辆状态
- 发布 `/line_planned_path` 话题输出规划路径
- 发布 `/line_finish_signal` 话题输出终点到达信号
- 离线模式：第一次检测后持续跟踪直至到达终点

## 编译

```bash
cd /home/kerwin/2025huat
catkin build line_detection
source devel/setup.bash
```

## 运行

### 基本运行

```bash
roslaunch line_detection line_detection.launch
```

### 使用配置文件

```bash
roslaunch line_detection line_detection.launch
rosparam load /home/kerwin/2025huat/src/planning/line_detection/config/line_detection.yaml
```

## 参数配置

所有参数都在 `config/line_detection.yaml` 中配置：

### 霍夫变换参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| hough/rho_resolution | 0.1 | ρ分辨率（米） |
| hough/theta_resolution | 0.01 | θ分辨率（弧度） |
| hough/min_votes | 3 | 最小投票数 |
| hough/theta_tolerance | 0.2 | 角度容差（弧度） |

### 锥桶过滤参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| cones/max_distance | 50.0 | 最大锥桶距离（米） |
| cones/min_distance | 2.0 | 最小锥桶距离（米） |

### 路径生成参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| path/interval | 0.1 | 路径点间隔（米） |
| path/max_distance | 75.0 | 最大路径距离（米） |

### 车辆参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| vehicle/imu_offset_x | 1.88 | IMU到激光雷达的X偏移（米） |

### 终点检测

| 参数 | 默认值 | 说明 |
|------|--------|------|
| finish/threshold | 2.0 | 终点检测距离阈值（米） |

## 话题

### 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| /cone_position | common_msgs/Cone | 锥桶位置 |
| /Carstate | common_msgs/HUAT_Carstate | 车辆状态 |

### 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| /line_planned_path | nav_msgs/Path | 规划路径 |
| /line_finish_signal | std_msgs/Bool | 终点到达信号 |

## 依赖

- ROS Noetic
- common_msgs
- geometry_msgs
- nav_msgs
- sensor_msgs
- std_msgs
- Eigen3
- roscpp

## 文件结构

```
line_detection/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── line_detection.yaml
├── include/line_detection/
│   └── line_detection.hpp
├── launch/
│   └── line_detection.launch
└── src/
    ├── line_detection.cpp
    └── main.cpp
```

## 故障排除

### 编译错误

如果遇到编译错误，请确保：
1. 已正确安装所有依赖包
2. 工作空间已正确配置
3. 运行 `catkin clean line_detection` 后重新编译

### 运行时错误

1. 检查 `/cone_position` 话题是否正常发布
2. 检查 `/Carstate` 话题是否正常发布
3. 查看ROS日志获取详细错误信息

## 作者

FSAC Racing Team

## 许可证

MIT
