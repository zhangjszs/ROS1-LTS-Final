# 车辆几何参数管理规范

## 概述

车辆几何参数是多个子系统共享的关键配置，包括轴距、质量、重心位置等。
本文档规范化参数管理策略，确保单一真值源和一致性。

## 设计原则

1. **单一真值源（Single Source of Truth）**：所有车辆几何参数集中在一个文件中定义
2. **全局命名空间**：由顶层launch文件加载到全局命名空间
3. **命名空间前缀**：各子系统通过命名空间前缀读取参数
4. **Fallback机制**：子系统配置文件中保留默认值作为fallback

## 参数定义位置

### 统一真值源

**文件路径**: `fsd_launch/config/vehicles/{vehicle_name}/vehicle_geometry.yaml`

示例: `fsd_launch/config/vehicles/A13/vehicle_geometry.yaml`

**加载位置**: `fsd_launch/launch/subsystems/mission_stack.launch`

```xml
<!-- Vehicle geometry: 统一真值源，在子系统之前加载 -->
<arg name="vehicle_geometry_config"
     default="$(find fsd_launch)/config/vehicles/$(arg vehicle)/vehicle_geometry.yaml"/>
<rosparam command="load" file="$(arg vehicle_geometry_config)"/>
```

### Fallback默认值

各子系统配置文件中保留默认值，仅在vehicle_geometry.yaml未加载时生效：

- `localization_ros/config/state_estimator.yaml`
- `localization_ros/config/location.yaml`
- `control_ros/config/param.yaml`
- `simulation_ros/config/vehicle.yaml`

## 参数命名空间

### Control子系统 (`car_arg/`)

```yaml
car_arg:
  length: 1.55        # 轴距 [m]
  front_axle: 0.5     # 前轴到车头 [m]
  rear_axle: 1.05     # 后轴到车尾 [m]
  delta_max: 0.4      # 最大转向角 [rad]
  delta_min: 0.0174   # 最小转向角 [rad]
  cg_to_front: 0.77   # 重心到前轴 [m]
  cg_to_rear: 0.78    # 重心到后轴 [m]
  mass: 190.0         # 车辆质量 [kg]
```

**C++读取示例**:
```cpp
ros::NodeHandle pnh("~");
double wheelbase;
pnh.param<double>("/car_arg/length", wheelbase, 1.55);
```

### Localization子系统 (`kinematic_correction/`)

```yaml
kinematic_correction:
  wheelbase: 1.55     # 轴距 [m]
  cg_to_rear: 0.78    # 重心到后轴 [m]
```

**C++读取示例**:
```cpp
ros::NodeHandle nh;
double wheelbase;
nh.param<double>("/kinematic_correction/wheelbase", wheelbase, 1.55);
```

### Simulation子系统 (`vehicle/`)

```yaml
vehicle:
  mass: 190.0         # 车辆质量 [kg]
  wheelbase: 1.55     # 轴距 [m]
  cg_to_front: 0.77   # 重心到前轴 [m]
  cg_to_rear: 0.78    # 重心到后轴 [m]
```

## 参数定义

### 几何参数

| 参数名 | 单位 | 说明 | 典型值 |
|--------|------|------|--------|
| `wheelbase` / `length` | m | 前后轴距离 | 1.55 |
| `front_axle` | m | 前轴到车头距离 | 0.5 |
| `rear_axle` | m | 后轴到车尾距离 | 1.05 |
| `cg_to_front` | m | 重心到前轴距离 | 0.77 |
| `cg_to_rear` | m | 重心到后轴距离 | 0.78 |

### 质量参数

| 参数名 | 单位 | 说明 | 典型值 |
|--------|------|------|--------|
| `mass` | kg | 车辆总质量（含驾驶员） | 190.0 |

### 转向参数

| 参数名 | 单位 | 说明 | 典型值 |
|--------|------|------|--------|
| `delta_max` | rad | 最大转向角 | 0.4 |
| `delta_min` | rad | 最小转向角 | 0.0174 |

## 参数标定流程

### 1. 几何测量

**工具**: 卷尺、激光测距仪

**测量项目**:
- 前后轴中心距离（轴距）
- 前轴到车头最前端距离
- 后轴到车尾最后端距离
- 车辆宽度

**精度要求**: ±5mm

### 2. 质量测量

**工具**: 地磅、轴重秤

**测量项目**:
- 车辆总质量（含驾驶员）
- 前轴载荷
- 后轴载荷

**计算重心位置**:
```
cg_to_rear = (前轴载荷 / 总质量) × 轴距
cg_to_front = 轴距 - cg_to_rear
```

### 3. 转向角标定

**工具**: 转向角传感器、量角器

**测量项目**:
- 最大左转向角
- 最大右转向角
- 转向角传感器零点

### 4. 参数验证

**方法**: 实车测试

**验证项目**:
- 最小转弯半径测试
- 运动学模型验证
- 动力学模型验证

## 修改流程

### 1. 修改参数

编辑 `fsd_launch/config/vehicles/{vehicle_name}/vehicle_geometry.yaml`

### 2. 验证一致性

检查所有命名空间的参数是否一致：

```bash
# 检查参数一致性
rosparam get /car_arg/length
rosparam get /kinematic_correction/wheelbase
rosparam get /vehicle/wheelbase
```

### 3. 更新Fallback值

如果修改了关键参数，同步更新各子系统配置文件中的fallback默认值：

- `localization_ros/config/state_estimator.yaml`
- `localization_ros/config/location.yaml`

### 4. 测试验证

```bash
# 启动系统
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag

# 检查参数加载
rosparam list | grep -E "car_arg|kinematic_correction|vehicle"

# 验证运动学模型
rostopic echo /localization/car_state
```

## 常见问题

### Q1: 为什么需要多个命名空间？

**A**: 不同子系统使用不同的参数命名约定，为了兼容现有C++代码，保留各自的命名空间。

### Q2: 如何确保参数一致性？

**A**:
1. 所有参数在vehicle_geometry.yaml中定义（单一真值源）
2. 子系统配置文件中的值仅作为fallback
3. 启动时检查参数是否正确加载

### Q3: 修改参数后需要重新编译吗？

**A**: 不需要。参数是运行时加载的，修改后重新启动launch文件即可。

### Q4: 如何添加新的车辆配置？

**A**:
1. 创建新目录: `fsd_launch/config/vehicles/{new_vehicle_name}/`
2. 复制A13的vehicle_geometry.yaml
3. 修改参数值
4. 启动时指定: `vehicle:={new_vehicle_name}`

## 参数来源

### 设计值

来源于车辆设计图纸和CAD模型：
- 轴距
- 前后悬长度
- 车辆外形尺寸

### 测量值

来源于实车测量和标定：
- 实际轴距（考虑悬挂压缩）
- 质量分布
- 重心位置

### 标定值

来源于实车测试和参数辨识：
- 转向角限制
- 动力学参数
- 轮胎参数

## 版本历史

### v1.0 (2024-01-15)
- 初始版本
- 定义A13车辆参数

### v1.1 (2024-06-20)
- 统一质量参数: 250kg → 190kg
- 统一重心位置: 0.775m → 0.78m

### v1.2 (2026-02-18) - Task #11
- 添加详细文档说明
- 明确单一真值源原则
- 添加fallback机制说明
- 补充参数标定流程

## 参考资料

- [ROS Parameter Server](http://wiki.ros.org/Parameter%20Server)
- [Vehicle Dynamics and Control (Rajamani)](https://www.springer.com/gp/book/9781461414322)
- CLAUDE.md - Launch File Organization
- TRACK_SPECS.md - Track and Vehicle Constraints
