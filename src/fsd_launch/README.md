# FSD Launch 统一启动包

提供按任务/模式组织的 launch 文件，简化系统启动。

## 目录结构

```
fsd_launch/
├── launch/
│   ├── *.launch            # 任务入口（完整参数）
│   │   ├── trackdrive.launch    # 高速循迹
│   │   ├── skidpad.launch       # 八字绕环
│   │   ├── acceleration.launch  # 加速赛
│   │   ├── autocross.launch     # 障碍赛
│   │   └── ebs_test.launch      # EBS 接口预留
│   │
│   ├── *_real.launch       # 场景预设入口（实车）
│   ├── *_sim.launch        # 场景预设入口（回放）
│   │   ├── trackdrive_real.launch / trackdrive_sim.launch
│   │   ├── skidpad_real.launch / skidpad_sim.launch
│   │   ├── acceleration_real.launch / acceleration_sim.launch
│   │   ├── autocross_real.launch / autocross_sim.launch
│   │   ├── ebs_real.launch / ebs_sim.launch
│   │   └── sensing_real.launch / sensing_sim.launch
│
│   ├── subsystems/         # 子系统（内部使用）
│   │   ├── perception.launch    # 感知
│   │   ├── localization.launch  # 定位
│   │   ├── planning.launch      # 规划
│   │   ├── control.launch       # 控制
│   │   └── vehicle.launch       # 车辆接口
│   │
│   └── tools/              # 工具
│       ├── rosbag_play.launch   # Bag 回放
│       ├── rviz.launch          # 可视化
│       ├── topic_bridge.launch  # 话题桥接
│       └── debug.launch         # 调试模式
│
└── config/                 # 配置文件
    └── debug_console.conf
```

## 快速使用（推荐：预设入口）

### LiDAR + Vision + IMU（不启规划/控制）

```bash
# 实车
roslaunch fsd_launch sensing_real.launch

# 仿真/回放
roslaunch fsd_launch sensing_sim.launch bag:=/path/to/bag.bag
```

### 高速循迹（Trackdrive）

```bash
# 实车
roslaunch fsd_launch trackdrive_real.launch

# 仿真（rosbag 回放）
roslaunch fsd_launch trackdrive_sim.launch bag:=/path/to/bag.bag
```

### 八字绕环 / 加速赛 / 障碍赛 / EBS

```bash
roslaunch fsd_launch skidpad_real.launch
roslaunch fsd_launch acceleration_real.launch
roslaunch fsd_launch autocross_real.launch
roslaunch fsd_launch ebs_real.launch
```

## 进阶使用（完整参数入口）

### 高速循迹

```bash
# 实车
roslaunch fsd_launch trackdrive.launch

# 仿真（rosbag 回放）
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag
```

### 八字绕环

```bash
roslaunch fsd_launch skidpad.launch simulation:=true bag:=/path/to/bag.bag
```

### 加速赛

```bash
roslaunch fsd_launch acceleration.launch simulation:=true bag:=/path/to/bag.bag
```

### EBS 接口预留

```bash
# 默认仅跑感知/定位，不下发控制
roslaunch fsd_launch ebs_test.launch simulation:=true bag:=/path/to/bag.bag

# 显式开启控制（保留接口）：control_mode=5
roslaunch fsd_launch ebs_test.launch simulation:=true bag:=/path/to/bag.bag enable_ebs_control:=true control_mode:=5
```

### 调试模式

```bash
roslaunch fsd_launch debug.launch mission:=trackdrive bag:=/path/to/bag.bag
```

## 预设入口常用参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `bag` | "" | Rosbag 文件路径（`*_sim.launch` 必填） |
| `rate` | 1.0 | 回放速率（`*_sim.launch`） |
| `loop` | false | rosbag 循环回放（`*_sim.launch`） |
| `vehicle` | A13 | 车辆型谱ID |
| `launch_rviz` | true | 启动 RViz |
| `launch_viz` | true | 启动可视化节点 |
| `vision_image_topic` | /camera/image_raw | 视觉输入图像话题（支持的预设） |

## 子系统单独启动

```bash
# 只启动感知
roslaunch fsd_launch launch/subsystems/perception.launch

# 只启动规划（指定规划器）
roslaunch fsd_launch launch/subsystems/planning.launch planner:=high_speed
```
