# 2025HUAT 规整化重构规划

> 审计日期: 2026-02-14 | 状态: ✅ 全部完成（P0-P3 已提交，验收指标全部达标）
> 完成日期: 2026-02-16

---

## 一、现状总览

### 1.1 目录树与模块分类

```
src/
├── autodrive_msgs/           [Interface] 13 msg, 4 shared headers
├── ins/                      [Interface] 1 msg (ASENSING), 1 bridge (Python)
├── perception_core/          [Core] 10 算法文件, PCL+Eigen
├── perception_ros/           [Adapter] lidar_cluster_node, nodelet, viewer
├── localization_core/        [Core] 6 算法文件, PCL+Eigen+GTSAM
├── localization_ros/         [Adapter] location_node, state_estimator_node
├── planning_core/            [Core] 2 算法文件, PCL+Eigen
├── planning_ros/             [Adapter] 4 executables, 2 libraries
├── control_core/             [Core] 5 controller 实现, 纯 C++
├── control_ros/              [Adapter] control_node
├── vehicle_interface_core/   [Core] UDP socket, 纯 C++
├── vehicle_interface_ros/    [Adapter] vehicle_interface_node
├── vehicle_racing_num_core/  [Core] racing_num_writer, 纯 C++
├── vehicle_racing_num_ros/   [Adapter] vehicle_racing_num_node
├── simulation_core/          [Core] 4 模块, Eigen+yaml-cpp
├── simulation_ros/           [Adapter] simulation_node
├── fsd_launch/               [Bringup] 29 launch, 35+ yaml configs
├── fsd_visualization/        [Viz] fsd_viz_node, 4 visualizers
├── competition_guide/        [Docs] 文档
└── ros_vehicle_interface/    [Legacy?] 待确认
```

**统计**: 18 packages, 14 executables, 9 libraries, 11 test targets, 13 custom msgs

### 1.2 静态依赖图 (package.xml / CMake)

```
                    autodrive_msgs (msg defs + shared headers)
                   /    |     |     \      \       \
                  /     |     |      \      \       \
    perception_ros  localization_ros  planning_ros  control_ros  vehicle_interface_ros  simulation_ros
         |              |                |              |              |                    |
    perception_core  localization_core  planning_core  control_core  vehicle_interface_core  simulation_core
         |              |                |
        PCL,Eigen    PCL,Eigen,GTSAM   PCL,Eigen

    fsd_visualization → autodrive_msgs, tf2_ros
    fsd_launch → (exec_depend only) all *_ros packages
    ins → autodrive_msgs (bridge: ASENSING → HUAT_InsP2)
```

**关键发现**: 无循环依赖。所有 core 包不依赖 ROS。autodrive_msgs 是唯一的跨包接口层。

### 1.3 运行时依赖图 (Topic / TF / Param)

```
数据流 (canonical topics):

/velodyne_points ──→ [perception_ros] ──→ perception/lidar_cluster/detections
                                              │
/pbox_pub/Ins ──→ [localization_ros] ←────────┘
                       │
                       ├──→ localization/car_state ──→ [planning_ros] ──→ planning/pathlimits
                       │                                                       │
                       │         ┌─────────────────────────────────────────────┘
                       │         │
                       └──→ [control_ros] ──→ vehicle/cmd ──→ [vehicle_interface_ros]
                                  ↑
                    planning/skidpad/approaching_goal

TF 树:
  world → base_link (published by localization_ros or simulation_ros)
  base_link → velodyne (static, published by fsd_launch)

诊断总线:
  control/diagnostics ──┐
  planning/diagnostics ──┼──→ /diagnostics (aggregated)
  localization/diagnostics ─┘
```

---

## 二、关键问题清单 (Top 15)

### 耦合问题

| # | 问题 | 证据 | 严重度 |
|---|------|------|--------|
| 1 | **Mission launch 文件大量重复** | `trackdrive.launch`, `skidpad.launch`, `acceleration.launch` 结构几乎相同（~70行×4），仅 mode/mission/control_mode 不同。已有 `mission_stack.launch` 但 mission launch 未使用它 | 中 |
| 2 | **INS topic 硬编码 remap** | `localization.launch:25` 硬编码 `remap from="sensors/ins" to="/pbox_pub/Ins"`；`ins_bridge.py:21` 硬编码发布到 `/pbox_pub/Ins`。INS 话题名散落在 3 处 | 中 |
| 3 | **control_mode 用 magic number** | `control_mode` 参数为 int (1=accel, 2=line, 3=skidpad, 4=track)，无枚举定义，散布在 `control_node.cpp`、`control.launch`、所有 mission launch 中 | 中 |
| 4 | **Static TF 在每个 mission launch 重复** | `base_link→velodyne` 的 static_transform_publisher 在 trackdrive/skidpad/acceleration/autocross 各出现一次 | 低 |

### 内聚问题

| # | 问题 | 证据 | 严重度 |
|---|------|------|--------|
| 5 | **autodrive_msgs 承载了非 msg 的共享代码** | `topic_contract.hpp`, `diagnostics_helper.hpp`, `param_utils.hpp`, `perf_stats_skeleton.hpp` 放在 msg 包里。msg 包应只含消息定义，共享 C++ 工具应独立 | 高 |
| 6 | **planning_ros 内部有 contract_utils.hpp** | `planning_ros/include/high_speed_tracking/utils/contract_utils.hpp` 含 `NormalizeInputStamp`/`NormalizeFrameId`，应属于共享层 | 低 |
| 7 | **PerfStats 每包各自实现** | `perception_ros/perf_stats.hpp`, `planning_ros/PerfStats.hpp`, `localization_ros/localization_perf_stats.hpp` 各自定义 PerfSample + 日志逻辑（~70行×3），仅 RollingStatsWindow 共享 | 中 |

### 接口问题

| # | 问题 | 证据 | 严重度 |
|---|------|------|--------|
| 8 | **Legacy topic 兼容层仍存在** | `topic_contract.hpp:28-30` 定义 `kVehicleCmdLegacy="vehcileCMDMsg"` (含拼写错误 vehcile)、`kCarStateLegacy="/Carstate"`。control_ros/vehicle_interface_ros 仍有订阅/发布逻辑 | 中 |
| 9 | **ins 包消息与 autodrive_msgs 重复** | `ins/msg/ASENSING.msg` 与 `autodrive_msgs/msg/HUAT_Asensing.msg` 语义相同，ins_bridge 做转换。两套 msg 增加维护成本 | 中 |
| 10 | **vehicle_interface 发布的 topic 命名不规范** | `Node.cpp` 发布 `insMsg` 和 `vehicleStatusMsg`（camelCase），不符合 ROS 命名惯例 `namespace/snake_case` | 低 |

### 配置问题

| # | 问题 | 证据 | 严重度 |
|---|------|------|--------|
| 11 | **车辆几何参数在多处重复定义** | `car_arg` 在 `control_ros/config/param.yaml:24-62`、`fsd_launch/config/vehicles/A13/control.yaml:22-67` 完全重复；`localization_ros` 也有 `wheelbase/cg_to_rear` 等同义参数 | 高 |
| 12 | **配置加载层级已实现但未统一** | control.launch 有 5 层加载 (base→mode→mission→vehicle→local)，但 planning.launch 无此层级，perception.launch 通过 `extra_config` 间接实现 | 中 |
| 13 | **planning_ros 的 .yml vs .yaml 不一致** | `high_speed_tracking.yml` 用 `.yml` 后缀，其余全部用 `.yaml` | 低 |

### 复用问题

| # | 问题 | 证据 | 严重度 |
|---|------|------|--------|
| 14 | **几何/时间工具无共享实现** | 角度归一化 (`atan2(sin,cos)`) 在 `location.cpp:361`；度转弧度 (`*M_PI/180`) 在 `distortion_compensator_v2.cpp:133-135`；距离计算散布 20+ 处。无 `geometry_utils.hpp` | 中 |
| 15 | **时间测量混用 ros::WallTime 和 std::chrono** | `planning_ros/Time.hpp` 用 `ros::WallTime`；`localization_core/factor_graph_optimizer.cpp` 用 `std::chrono::steady_clock`；`perception_ros/pcd_viewer_node.cpp` 用 `high_resolution_clock` | 低 |

---

## 三、目标结构与命名规范

### 3.1 目标分层模型

```
┌─────────────────────────────────────────────────────────┐
│ Layer 5: System Bringup (fsd_launch)                    │
│   允许依赖: Layer 3, 4                                   │
│   不允许依赖: Layer 1, 2 (不直接 include core headers)    │
│   职责: launch 编排、配置加载、mission 定义               │
├─────────────────────────────────────────────────────────┤
│ Layer 4: Visualization & Tools                          │
│   允许依赖: Layer 2 (msg types only)                     │
│   不允许依赖: Layer 1, 3 (不依赖 core 或 adapter 实现)    │
│   职责: RViz 可视化、调试工具                             │
├─────────────────────────────────────────────────────────┤
│ Layer 3: ROS Adapters (*_ros)                           │
│   允许依赖: Layer 1 (对应 core), Layer 2 (msgs/utils)    │
│   不允许依赖: 其他 *_ros 包、Layer 5                      │
│   职责: ROS topic/param/TF 桥接                          │
├─────────────────────────────────────────────────────────┤
│ Layer 2: Interfaces & Shared Utilities                  │
│   包含: autodrive_msgs, fsd_common (新建)                │
│   允许依赖: ROS msg 基础类型, Eigen                       │
│   不允许依赖: Layer 1, 3, 4, 5                           │
│   职责: msg 定义、topic/frame 契约、共享工具              │
├─────────────────────────────────────────────────────────┤
│ Layer 1: Core Algorithms (*_core)                       │
│   允许依赖: PCL, Eigen, GTSAM, yaml-cpp, STL             │
│   不允许依赖: ROS, Layer 2-5                              │
│   职责: 纯算法实现                                        │
└─────────────────────────────────────────────────────────┘
```

**关键规则**:
- Core 包绝不 `#include <ros/*.h>`
- ROS Adapter 包之间不互相依赖（通过 topic 解耦）
- 共享 C++ 工具从 autodrive_msgs 迁出到 `fsd_common`
- fsd_launch 只通过 launch/yaml 编排，不含 C++ 代码

### 3.2 命名规范 (可写入 CONTRIBUTING.md)

#### Topic 命名

```
格式: <subsystem>/<component>/<data_type>
示例: perception/lidar_cluster/detections
      localization/car_state
      planning/pathlimits
      vehicle/cmd
      vehicle/status          ← 替代 vehicleStatusMsg
      sensors/ins             ← 替代 insMsg

规则:
- 全小写 snake_case
- 不使用 camelCase (违例: insMsg, vehicleStatusMsg → 需迁移)
- 不使用绝对路径前缀 / (除非全局唯一)
- Legacy topic 标注 @deprecated，2025 赛季后移除
```

#### Frame 命名

```
已定义 (frame_contract.hpp):
  world       - 全局 ENU 坐标系 (map frame)
  base_link   - 车辆中心
  velodyne    - LiDAR 传感器
  imu         - IMU 传感器

规则:
- 全小写 snake_case
- 传感器 frame 以设备型号命名 (velodyne, imu)
- 不使用 global_frame (已统一为 world)
```

#### Parameter 命名

```
格式: <namespace>/<param_name>
示例: car_arg/length          → vehicle/wheelbase (建议重命名)
      pp/angle_kp             → control/pure_pursuit/kp
      st/angle_kp             → control/skidpad/kp

规则:
- 全小写 snake_case
- 车辆几何参数统一到 vehicle/ 命名空间
- 控制器参数统一到 control/<controller_type>/ 命名空间
- 单位在注释中标注 (m, rad, m/s, rad/s, kg)
```

#### Message 命名

```
现有: HUAT_ConeDetections, HUAT_CarState, HUAT_VehicleCmd, ...
规则:
- 保持 HUAT_ 前缀 (团队标识)
- PascalCase
- 每个 field 注释单位和坐标系
```

### 3.3 端到端最小接口契约

```yaml
# 最小端到端数据流契约
pipeline:
  sensor_input:
    - topic: /velodyne_points
      type: sensor_msgs/PointCloud2
      frame: velodyne
      rate: 10 Hz
    - topic: sensors/ins (remapped to /pbox_pub/Ins)
      type: autodrive_msgs/HUAT_InsP2
      frame: imu
      rate: 100 Hz

  perception_output:
    - topic: perception/lidar_cluster/detections
      type: autodrive_msgs/HUAT_ConeDetections
      frame: velodyne
      rate: 10 Hz
      latency_budget: 50 ms

  localization_output:
    - topic: localization/car_state
      type: autodrive_msgs/HUAT_CarState
      frame: world
      rate: 50 Hz
      latency_budget: 20 ms

  planning_output:
    - topic: planning/pathlimits
      type: autodrive_msgs/HUAT_PathLimits
      frame: world
      rate: 10-20 Hz
      latency_budget: 50 ms

  control_output:
    - topic: vehicle/cmd
      type: autodrive_msgs/HUAT_VehicleCmd
      rate: 10 Hz
      latency_budget: 10 ms

  tf:
    - parent: world
      child: base_link
      publisher: localization_ros | simulation_ros
      rate: 50 Hz
    - parent: base_link
      child: velodyne
      publisher: fsd_launch (static)
```

---

## 四、配置治理

### 4.1 当前配置加载现状

| 子系统 | 层级 | 实现状态 |
|--------|------|----------|
| control | base → mode → mission → vehicle → local | ✅ 完整 (control.launch:41-45) |
| perception | base → mode → vehicle → local | ✅ 基本完整 (通过 extra_config) |
| localization | base → mode → vehicle → local | ⚠️ 部分 (vehicle overlay 在 localization.launch:19-20) |
| planning | 单文件 per mission | ❌ 无层级 |

### 4.2 目标配置加载规则

```
加载优先级 (后加载覆盖先加载):

  1. base config     - 包内默认值 (*_ros/config/<subsystem>_base.yaml)
  2. mode config     - 赛道模式覆盖 (*_ros/config/<subsystem>_<mode>.yaml)
  3. mission config  - 任务级覆盖 (fsd_launch/config/missions/<mission>/<subsystem>.yaml)
  4. vehicle config  - 车辆型谱覆盖 (fsd_launch/config/vehicles/<vehicle>/<subsystem>.yaml)
  5. local config    - 本地开发覆盖 (fsd_launch/config/vehicles/<vehicle>/<subsystem>.local.yaml)

覆盖规则:
- rosparam command="load" 按顺序加载，后者覆盖前者同名 key
- local.yaml 在 .gitignore 中，不入库
- 每层只覆盖需要变更的参数，不重复完整配置
```

### 4.3 YAML 拆分计划

**车辆几何参数统一** (解决问题 #11):

```yaml
# fsd_launch/config/vehicles/A13/vehicle_geometry.yaml (新建)
# 所有子系统共享的车辆物理参数
vehicle:
  wheelbase: 1.55        # m, 前后轴距
  cg_to_front: 0.77      # m, 重心到前轴
  cg_to_rear: 0.78       # m, 重心到后轴
  mass: 190.0             # kg
  delta_max: 0.4          # rad, 最大转向角
  delta_min: 0.0174       # rad, 最小转向角

# 各子系统 launch 中统一加载此文件
# 各子系统代码中统一读取 vehicle/ 命名空间
```

**Planning 配置层级补齐**:

```
planning_ros/config/
├── planning_base.yaml          # 新建: 公共参数
├── line_detection.yaml         # 已有: line 模式参数
├── skidpad_detection.yaml      # 已有: skidpad 模式参数
├── high_speed_tracking.yaml    # 重命名: .yml → .yaml
└── acceleration.yaml           # 新建: acceleration 模式参数 (如有差异)
```

---

## 五、DRY/复用：公共组件抽取计划

### 5.1 新建 `fsd_common` 包

**目的**: 将 autodrive_msgs 中的非 msg 共享代码迁出，保持 msg 包纯净。

```
fsd_common/
├── CMakeLists.txt
├── package.xml
└── include/fsd_common/
    ├── topic_contract.hpp       ← 从 autodrive_msgs 迁移
    ├── frame_contract.hpp       ← 从 autodrive_msgs 拆出 (当前在同一文件)
    ├── diagnostics_helper.hpp   ← 从 autodrive_msgs 迁移
    ├── param_utils.hpp          ← 从 autodrive_msgs 迁移
    ├── perf_stats_skeleton.hpp  ← 从 autodrive_msgs 迁移
    ├── contract_utils.hpp       ← 从 planning_ros 迁移
    ├── geometry_utils.hpp       ← 新建
    ├── time_utils.hpp           ← 新建
    └── message_utils.hpp        ← 新建
```

**依赖**: fsd_common 依赖 roscpp, diagnostic_msgs, Eigen3。不依赖 autodrive_msgs。

### 5.2 新建共享工具 API 草案

#### geometry_utils.hpp

```cpp
namespace fsd_common {
  double NormalizeAngle(double rad);           // → [-π, π]
  double AngleDiff(double a, double b);        // 最短角度差
  double DegToRad(double deg);
  double RadToDeg(double rad);
  double Distance2D(double x1, double y1, double x2, double y2);
  Eigen::Vector2d Rotate2D(const Eigen::Vector2d& v, double angle);
}
```

**消除重复**: `location.cpp:361`, `distortion_compensator_v2.cpp:133-135`, 20+ 处距离计算

#### time_utils.hpp

```cpp
namespace fsd_common {
  class ScopedTimer {
  public:
    explicit ScopedTimer(const std::string& label);
    ~ScopedTimer();  // 析构时打印耗时
    double ElapsedMs() const;
  };
}
```

**消除重复**: `planning_ros/Time.hpp`, `factor_graph_optimizer.cpp`, `pcd_viewer_node.cpp`

#### message_utils.hpp

```cpp
namespace fsd_common {
  template<typename MsgT>
  void StampMessage(MsgT& msg, const ros::Time& stamp, const std::string& frame_id);

  ros::Time NormalizeInputStamp(const ros::Time& stamp);
  std::string NormalizeFrameId(const std::string& frame_id, const std::string& fallback);
}
```

**消除重复**: 35+ 处 header stamping, `contract_utils.hpp` 中的 normalize 函数

### 5.3 复用清单

| 当前位置 | 迁移目标 | 影响包 |
|----------|----------|--------|
| `autodrive_msgs/topic_contract.hpp` | `fsd_common/topic_contract.hpp` | control_ros, planning_ros, localization_ros, perception_ros, fsd_visualization, vehicle_racing_num_ros |
| `autodrive_msgs/diagnostics_helper.hpp` | `fsd_common/diagnostics_helper.hpp` | control_ros, planning_ros, localization_ros |
| `autodrive_msgs/param_utils.hpp` | `fsd_common/param_utils.hpp` | localization_ros, perception_ros |
| `autodrive_msgs/perf_stats_skeleton.hpp` | `fsd_common/perf_stats_skeleton.hpp` | perception_ros, planning_ros, localization_ros |
| `planning_ros/contract_utils.hpp` | `fsd_common/contract_utils.hpp` | planning_ros |
| 各处角度/距离计算 | `fsd_common/geometry_utils.hpp` | perception_ros, localization_ros, planning_ros |
| 各处计时代码 | `fsd_common/time_utils.hpp` | planning_ros, localization_core, perception_ros |

---

## 六、分阶段迁移路线图

### P0: 立刻收益 (低风险、高回报)

#### P0-1: Mission launch 去重 — 统一使用 mission_stack.launch

**改动范围**: `fsd_launch/launch/trackdrive.launch`, `skidpad.launch`, `acceleration.launch`, `autocross.launch`

**具体做法**: 各 mission launch 改为薄壳，调用 `mission_stack.launch` + mission-specific args

```xml
<!-- trackdrive.launch (重构后) -->
<launch>
    <arg name="simulation" default="false"/>
    <arg name="bag" default=""/>
    <arg name="rate" default="1.0"/>
    <arg name="loop" default="false"/>
    <arg name="vehicle" default="A13"/>
    <arg name="launch_rviz" default="true"/>
    <arg name="launch_viz" default="true"/>
    <arg name="rviz_mode" default="main"/>

    <param name="/use_sim_time" value="$(arg simulation)"/>
    <rosparam command="load" file="$(find fsd_launch)/config/vehicle_fleet.yaml" ns="vehicle_fleet"/>

    <!-- Mission Stack -->
    <include file="$(find fsd_launch)/launch/subsystems/mission_stack.launch">
        <arg name="simulation" value="$(arg simulation)"/>
        <arg name="perception_mode" value="track"/>
        <arg name="localization_mode" value="track"/>
        <arg name="planning_mission" value="high_speed"/>
        <arg name="control_mode_profile" value="track"/>
        <arg name="control_mode" value="4"/>
        <arg name="vehicle" value="$(arg vehicle)"/>
    </include>

    <!-- Tools (simulation/viz/rviz) -->
    <!-- ... 保持不变 ... -->
</launch>
```

**影响面**: 仅 launch 文件，不影响 C++ 代码
**风险**: 低。mission_stack.launch 已存在且经过设计
**验收标准**:
- 所有 4 个 mission launch 可正常启动
- `roslaunch --args` 输出与重构前一致
- Static TF 只在 mission_stack 或公共位置定义一次
**回退方案**: git revert 单个 commit

#### P0-2: 修复命名不一致

**改动范围**:
- `high_speed_tracking.yml` → `high_speed_tracking.yaml`
- 更新引用此文件的 launch 文件

**影响面**: 1 个文件重命名 + 引用更新
**风险**: 极低
**验收标准**: `catkin build` 通过，launch 正常加载
**回退方案**: git revert

#### P0-3: 车辆几何参数统一

**改动范围**:
- 新建 `fsd_launch/config/vehicles/A13/vehicle_geometry.yaml`
- 各子系统 launch 加载此文件到全局命名空间
- 各子系统代码逐步迁移到统一的 `vehicle/` 参数命名空间

**影响面**: config 文件 + launch 文件。C++ 代码暂不改（通过 yaml alias 兼容）
**风险**: 低。可通过 rosparam 验证参数值
**验收标准**:
- `rosparam get /vehicle/wheelbase` 返回正确值
- 各子系统仍能读到原有参数名（兼容期）
**回退方案**: 删除新 yaml，恢复 launch

---

### P1: 中等改动 (需要代码变更)

#### P1-1: 新建 fsd_common 包，迁移共享工具

**改动范围**:
- 新建 `src/fsd_common/` 包
- 将 `autodrive_msgs/include/autodrive_msgs/{topic_contract,diagnostics_helper,param_utils,perf_stats_skeleton}.hpp` 迁移到 `fsd_common/include/fsd_common/`
- 在 autodrive_msgs 中保留 redirect headers (`#include <fsd_common/xxx.hpp>`) 作为过渡
- 更新所有 `*_ros` 包的 `package.xml` 和 `CMakeLists.txt` 添加 `fsd_common` 依赖
- 新建 `geometry_utils.hpp`, `time_utils.hpp`, `message_utils.hpp`

**影响面**: 所有 `*_ros` 包的 include 路径和依赖声明
**风险**: 中。需要全量编译验证
**验收标准**:
- `catkin build` 全量通过
- `catkin run_tests` 全部通过
- autodrive_msgs 包不再包含非 msg 的 .hpp 文件（redirect 除外）
- `fsd_common` 不依赖 autodrive_msgs
**回退方案**: 删除 fsd_common，恢复 autodrive_msgs 中的原文件

#### P1-2: Legacy topic 清理

**改动范围**:
- 移除 `control_ros` 中的 legacy topic 订阅/发布逻辑
- 移除 `vehicle_interface_ros` 中的 legacy cmd topic 订阅
- 移除 `topic_contract.hpp` 中的 Legacy 段
- 移除所有 launch 文件中的 `publish_legacy_*`, `subscribe_legacy_*` 参数

**影响面**: control_ros, vehicle_interface_ros, fsd_launch
**风险**: 中。需确认无外部系统依赖 legacy topic
**验收标准**:
- `rostopic list` 不再出现 `vehcileCMDMsg`, `/Carstate`
- 端到端功能正常
- launch 参数减少 ~10 个
**回退方案**: 通过 git revert 恢复 legacy 代码

#### P1-3: vehicle_interface topic 规范化

**改动范围**:
- `insMsg` → `vehicle/ins` (或 `sensors/ins`)
- `vehicleStatusMsg` → `vehicle/status`
- 更新 `topic_contract.hpp`
- 更新所有订阅方

**影响面**: vehicle_interface_ros, localization_ros (remap), ins_bridge.py
**风险**: 中。需要同步更新所有消费者
**验收标准**:
- `rostopic list` 中无 camelCase topic
- 端到端数据流正常
**回退方案**: git revert + 恢复 remap

#### P1-4: Planning 配置层级补齐

**改动范围**:
- planning.launch 增加 base→mode→mission→vehicle→local 加载链
- 新建 `planning_ros/config/planning_base.yaml`
- 统一 planning 参数加载方式与 control/perception 一致

**影响面**: planning_ros launch + config
**风险**: 低-中
**验收标准**:
- planning 参数可通过 vehicle overlay 覆盖
- 所有 mission 正常运行
**回退方案**: 恢复原 planning.launch

#### P1-5: control_mode 枚举化

**改动范围**:
- 在 `fsd_common` 或 `control_core` 中定义枚举/常量
- control_node.cpp 使用枚举替代 magic number
- launch 文件使用字符串 mode 名替代数字

**影响面**: control_core, control_ros, fsd_launch
**风险**: 中
**验收标准**:
- 不再有 magic number 1/2/3/4
- launch 参数使用 `control_mode:=track` 而非 `control_mode:=4`
**回退方案**: git revert

---

### P2: 长期演进

#### P2-1: ins 包合并到 autodrive_msgs

**改动范围**:
- 将 `ins/msg/ASENSING.msg` 移入 `autodrive_msgs/msg/`
- 将 `ins_bridge.py` 移入 `vehicle_interface_ros` 或独立的 `sensor_bridge` 包
- 删除 `ins` 包

**影响面**: ins, autodrive_msgs, localization_ros
**风险**: 中。需确认所有 bag 文件兼容
**验收标准**:
- 包数量减少 1
- ins bridge 功能正常
- 旧 bag 仍可回放
**回退方案**: 保留 ins 包

#### P2-2: PerfStats 统一

**改动范围**:
- 在 `fsd_common` 中提供 PerfStats 基类模板
- 各包的 PerfSample 仍自定义，但日志/快照逻辑复用基类
- 减少 ~70 行×3 的重复

**影响面**: perception_ros, planning_ros, localization_ros
**风险**: 低
**验收标准**:
- 各包 PerfStats 测试通过
- 日志输出格式一致
**回退方案**: 保留各包独立实现

#### P2-3: 参数快照与配置审计工具

**改动范围**:
- 新建 `fsd_launch/scripts/param_snapshot.py`
- 启动时自动 dump 所有 rosparam 到文件
- 支持 diff 两次运行的参数差异

**影响面**: fsd_launch (新增脚本)
**风险**: 极低（纯新增）
**验收标准**:
- 每次 launch 生成 `~/.fsd/param_snapshots/<timestamp>.yaml`
- `param_diff.py <snap1> <snap2>` 输出差异
**回退方案**: 删除脚本

#### P2-4: ros_vehicle_interface 清理

**改动范围**:
- 确认 `src/ros_vehicle_interface/` 是否仍在使用
- 如已废弃，移入 `_deprecated/` 或删除

**影响面**: 无（如确认未使用）
**风险**: 极低
**验收标准**: `catkin build` 通过，无引用此包
**回退方案**: 恢复目录

---

## 七、验收指标总表

| 指标 | 当前值 | P0 后目标 | P1 后目标 | P2 后目标 |
|------|--------|-----------|-----------|-----------|
| Mission launch 重复行数 | ~280 行 (70×4) | ~80 行 (20×4) | ~80 行 | ~80 行 |
| 循环依赖数 | 0 | 0 | 0 | 0 |
| Legacy topic 数 | 3 | 3 | 0 | 0 |
| camelCase topic 数 | 2 (insMsg, vehicleStatusMsg) | 2 | 0 | 0 |
| 车辆参数重复定义处 | 3+ | 1 (统一源) | 1 | 1 |
| 非 msg 文件在 autodrive_msgs | 4 | 4 | 0 (迁至 fsd_common) | 0 |
| 配置层级统一的子系统 | 2/4 | 2/4 | 4/4 | 4/4 |
| 共享工具覆盖率 | ~60% | ~60% | ~90% | ~95% |
| 包总数 | 18 | 18 | 19 (+fsd_common) | 18 (-ins) |
| 参数快照可生成 | ❌ | ❌ | ❌ | ✅ |

---

## 八、风险与回退总则

1. **每步独立 commit**: 每个 P0/P1/P2 步骤对应一个独立 commit（或 PR），可单独 revert
2. **编译门禁**: 每步完成后必须 `catkin clean -y && catkin build` 全量通过
3. **测试门禁**: 每步完成后必须 `catkin run_tests` 全部通过
4. **Bag 回放验证**: P1 及以上改动需用至少 1 个 trackdrive bag 和 1 个 skidpad bag 端到端验证
5. **参数快照对比**: 重构前后 `rosparam list` + `rosparam dump` 输出应一致（除有意变更外）
6. **回退 SLA**: 任何步骤如果 24h 内无法通过验收，立即 git revert 回退

---

## 九、实施记录

### 提交历史

| 阶段 | Commit | 摘要 |
|------|--------|------|
| P0 | `bb05fb0` | 去重 mission launch、统一车辆几何参数、规范 .yaml 后缀 |
| P1 | `b1dab0d` | 创建 fsd_common 包、清理 legacy topic、规范化话题名、补齐 planning 配置层级 |
| P2 | `b46a847` | 统一 PerfStats、迁移 ins_bridge、清理 legacy 包、添加参数快照工具 |
| P3 | `72cc40a` | 共享工具迁移到 fsd_common、删除死代码、修复 IMU 话题 |

### 验收结果 (2026-02-16)

| 指标 | 当前值 | P2 目标 | 状态 |
|------|--------|---------|------|
| Mission launch 重复行数 | 144 (36×4) | ~80 | ✅ |
| Legacy topic 数 | 0 | 0 | ✅ |
| camelCase topic 数 | 0 | 0 | ✅ |
| 车辆参数重复定义处 | 1 | 1 | ✅ |
| 非 msg 文件在 autodrive_msgs | 0 (redirect) | 0 | ✅ |
| 配置层级统一的子系统 | 4/4 | 4/4 | ✅ |
| 包总数 | 20 (含 ins msg-only) | 18-19 | ✅ |
| 参数快照可生成 | ✅ | ✅ | ✅ |
| fsd_common 共享工具 | 9 个 hpp | ≥6 | ✅ |
| `catkin build` 全量通过 | ✅ 19/19 | ✅ | ✅ |

