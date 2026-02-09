# FSD Launch 统一启动包

提供按任务/模式组织的 launch 文件，简化系统启动。

## 目录结构

```
fsd_launch/
├── launch/
│   ├── *.launch            # 按比赛任务
│   │   ├── trackdrive.launch    # 高速循迹
│   │   ├── skidpad.launch       # 八字绕环
│   │   ├── acceleration.launch  # 加速赛
│   │   ├── autocross.launch     # 障碍赛
│   │   └── ebs_test.launch      # EBS 接口预留
│   │
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

## 快速使用

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

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `simulation` | false | 仿真模式（使用 rosbag） |
| `bag` | "" | Rosbag 文件路径 |
| `rate` | 1.0 | 回放速率 |
| `launch_rviz` | true | 启动 RViz |
| `control_mode` | -1 | 控制算法模式ID（在 mission launch 中已按任务下发） |

## 子系统单独启动

```bash
# 只启动感知
roslaunch fsd_launch perception.launch

# 只启动规划（指定规划器）
roslaunch fsd_launch planning.launch planner:=high_speed
```
