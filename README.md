# 2025HUAT - FSD 无人驾驶系统

## 项目架构

本项目采用 **Core + ROS Wrapper** 分层架构，实现 ROS 与算法逻辑的解耦。

```text
src/
├── autodrive_msgs/          # 统一消息定义 (HUAT_ConeDetections, HUAT_CarState 等)
├── fsd_common/              # 核心通用类型与几何定义 (无 ROS 依赖)
├── ins/                     # INS 惯导传感器消息兼容桥接
├── fsd_launch/              # 统一启动配置中心 (任务入口/子系统/工具/仿真)
├── fsd_visualization/       # 统一可视化节点与 RViz 配置
├── perception_core/         # 激光雷达点云感知算法核心 (无 ROS 依赖)
├── perception_ros/          # 感知 ROS 包装层
├── vision_core/             # 视觉目标识别与回退核心 (无 ROS 依赖)
├── vision_ros/              # 视觉 ROS 包装层与模型推理
├── planning_core/           # 赛道规划算法核心 (无 ROS 依赖)
├── planning_ros/            # 规划 ROS 包装层
├── control_core/            # 横纵向控制算法核心 (无 ROS 依赖)
├── control_ros/             # 控制 ROS 包装层
├── localization_core/       # 因子图与滤波状态估计核心 (无 ROS 依赖)
├── localization_ros/        # 定位 ROS 包装层
├── simulation_core/         # 车辆动力学与轮胎仿真核心 (无 ROS 依赖)
├── simulation_ros/          # 仿真环境 ROS 节点与接口
├── vehicle_interface_core/  # 底盘 CAN/UDP 通信协议核心
├── vehicle_interface_ros/   # 底盘通信接口 ROS 包装层
├── vehicle_racing_num_core/ # 比赛车号管理核心
└── vehicle_racing_num_ros/  # 比赛车号 ROS 包装层
```
 
## 仓库目录导览
 
```text
.
├── src/                 # 核心 ROS1 Catkin 源码包 (21个包，Core + ROS Wrapper 解耦架构)
├── docs/                # 系统设计、接口契约、赛事规则与技术报告文档中心
├── scripts/             # 开发维护、CI门禁、回放测试与实车自启脚本
├── perf_reports/        # 自动化性能基准测试、图表生成与回归评估报告系统
├── .github/             # GitHub Actions CI/CD 流水线与协作模板
├── pyproject.toml       # Python 工具统一配置 (black, isort, pytest)
├── .clang-format        # C++17 代码规范格式化定义
├── CONTRIBUTING.md      # 开发者代码贡献与协作规范指南
├── CHANGELOG.md         # 版本发布与演进历史
└── LICENSE              # BSD-3-Clause 开源许可证
```
 
## 快速开始

### 构建
```bash
cd ~/2025huat
catkin build              # 构建所有包
source devel/setup.bash

# 构建特定包
catkin build <package_name>

# 清理并重新构建
catkin clean -y
catkin build
```

### 测试
```bash
# 运行所有测试
catkin run_tests

# 运行特定包的测试
catkin run_tests <package_name>
```

### 运行

**仿真模式（rosbag 回放）:**
```bash
# 基础仿真
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag

# 循环播放
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag loop:=true

# 自定义播放速率
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rate:=0.5
```

**实车模式:**
```bash
bash scripts/vehicle/autoStartGkj/start.sh
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

### entrypoints（任务入口）
- `src/fsd_launch/launch/trackdrive.launch`
- `src/fsd_launch/launch/skidpad.launch`
- `src/fsd_launch/launch/autocross.launch`
- `src/fsd_launch/launch/acceleration.launch`

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

> 说明：任务入口位于 `src/fsd_launch/launch/` 根目录，直接使用
> `roslaunch fsd_launch trackdrive.launch ...`

## TrackDrive 启动参数说明

示例（仿真 + 回放 + 双窗口 RViz + 循环）：
```bash
roslaunch fsd_launch trackdrive.launch \
  simulation:=true \
  bag:=/home/kerwin/rosbag/22910_2.bag \
  rviz_mode:=dual \
  loop:=true
```

参数说明（来自 `trackdrive.launch`）：

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
| `/perception/lidar_cluster/detections` | `autodrive_msgs/HUAT_ConeDetections` | 感知锥桶检测输出 |
| `/localization/car_state` | `autodrive_msgs/HUAT_CarState` | 统一车辆状态 |
| `/planning/pathlimits` | `autodrive_msgs/HUAT_PathLimits` | 规划输出路径约束 |
| `/vehicle/cmd` | `autodrive_msgs/HUAT_VehicleCmd` | 统一控制指令 |
| `/fsd/viz/*` | Marker/MarkerArray | 可视化标记 |

> 兼容链路仍可选 legacy 话题（默认关闭）：`/Carstate`、`/vehcileCMDMsg`。

## 技术规范

- **ROS 版本:** Noetic
- **C++ 标准:** C++17
- **构建工具:** catkin_tools
- **许可证:** BSD-3-Clause
- **包版本:** 1.0.0 (统一)

## Core + ROS Wrapper 架构优势

本项目采用分层架构，将算法核心与 ROS 中间件解耦：

- **可测试性:** Core 包可独立进行单元测试，无需 ROS 环境
- **可移植性:** 算法核心可移植到非 ROS 系统
- **清晰分离:** 算法逻辑与通信层职责明确
- **易维护:** 修改算法无需关注 ROS 细节，修改接口无需改动算法

## 开发指南

### 添加新功能

1. 在对应的 `*_core/` 包中实现算法（无 ROS 依赖）
2. 在 `*_core/test/` 中添加单元测试
3. 在对应的 `*_ros/` 包中创建 ROS 包装层
4. 根据需要在 `fsd_launch/` 中添加启动文件
5. 根据需要在 `fsd_visualization/` 中添加可视化

### 调试

```bash
# 使用调试工具启动
roslaunch fsd_launch tools/debug.launch mission:=trackdrive bag:=/path/to/bag.bag

# 检查话题
rostopic list
rostopic echo /perception/lidar_cluster/detections
rostopic hz /velodyne_points

# 检查坐标变换
rosrun tf tf_echo velodyne base_link
```

## 文档

- **[CLAUDE.md](CLAUDE.md)** - Claude Code 开发指南
- **[docs/README.md](docs/README.md)** - 统一技术文档中心导航索引
- **[scripts/README.md](scripts/README.md)** - 自动化与开发维护脚本全景字典
- **[perf_reports/README.md](perf_reports/README.md)** - 性能基准测试与报告系统说明
- **[docs/reports/remaining_work_audit_2026-03-06.md](docs/reports/remaining_work_audit_2026-03-06.md)** - 当前未完成事项与依赖审计基准
- **[docs/reports/HANDOFF.md](docs/reports/HANDOFF.md)** - 历史状态基线与交接说明
- **[docs/competition/](docs/competition/)** - FSAC 比赛规程与赛道/锥桶规范
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - 社区贡献与规范指南
- 各包 README - 查看 `src/*/README.md`

## 项目状态

![ROS CI](https://github.com/zhangjszs/ROS1-LTS-Final/workflows/ROS%20CI/badge.svg)
![Code Coverage](https://github.com/zhangjszs/ROS1-LTS-Final/workflows/Code%20Coverage/badge.svg)
![Static Analysis](https://github.com/zhangjszs/ROS1-LTS-Final/workflows/Static%20Analysis/badge.svg)

- ✅ 所有包构建通过
- ✅ 所有 core 包有单元测试
- ✅ 所有 core 包有文档
- ✅ 版本号统一为 1.0.0
- ✅ 许可证统一为 BSD-3-Clause
- ✅ 文件权限已规范化
- ✅ CI/CD 自动化测试
- ✅ 代码覆盖率报告
- ✅ 静态代码分析

## 联系方式

- 项目维护者: kerwin (zhangjszs@foxmail.com)
