# P2 任务文档化集合

> **完成时间**: 2026-03-02  
> 覆盖 Task #15/#16/#17/#18/#22

---

## Task #15: ConeDetections 颜色表示规范 ✅

### 颜色类型表 (`color_types` 字段)

| 值 | 枚举名 | 含义 | 使用场景 |
|----|--------|------|---------|
| `0` | `BLUE` | 蓝色锥桶 | 右边界 (高速绕环/加速) |
| `1` | `YELLOW` | 黄色锥桶 | 左边界 (高速绕环/加速) |
| `2` | `ORANGE_SMALL` | 小橙色锥桶 | 起点/终点标记 |
| `3` | `ORANGE_BIG` | 大橙色锥桶 | 起点/终点标记 |
| `4` | `NONE` | 未知/未分类 | LiDAR-only 默认值 |
| `5` | `RED` | 红色锥桶 | 左边界 (特定赛事) |

### 颜色流转路径

```
LiDAR感知 → ORANGE_SMALL/ORANGE_BIG/NONE (基于几何尺寸)
Camera视觉 → BLUE/YELLOW/RED (基于颜色分类)
融合 (vision_inject) → vision 颜色覆盖 LiDAR NONE
定位层 → 继承并存储到 point_types_[]
规划层 → 用于左右边界分类 (WayComputer/LineDetection/Skidpad)
```

### 使用约定

- **LiDAR-only 模式**: 所有锥桶默认为 `NONE(4)`，规划层回退到几何分割
- **视觉融合模式**: `vision_inject_enabled=true` 时，摄像头颜色注入 LiDAR 检测结果
- **插值锥桶**: 若两侧邻居颜色一致，继承该颜色；否则保持 `NONE`
- **越界处理**: 规划层遇到 `type > 5` 时，`ROS_WARN_ONCE` 并回退到几何路径

---

## Task #16: CarState 废弃字段替代关系 ✅

### HUAT_CarState 字段映射

| 废弃字段 | 替代字段 | 说明 |
|---------|---------|------|
| `car_state.x/y/theta` (Point3D) | `car_state.x/y`, `car_state.theta` | 直接使用，无废弃 |
| `W` (偏航角速度) | `Wz` (FSSIM 扩展) | `W` 仍由 location 填充；`Wz` 为冗余精确值 |
| `A` (加速度标量) | `Ax` / `Ay` | `A` 为合加速度；`Ax/Ay` 为分量，精度更高 |

### 推荐用法

```cpp
// 读取位置/航向 (规划/控制层)
const double x = state.car_state.x;
const double y = state.car_state.y;
const double theta = state.car_state.theta;

// 读取速度 (控制层)
const double v = static_cast<double>(state.V);       // 纵向速度 [m/s]
const double yaw_rate = static_cast<double>(state.Wz); // 偏航角速度 [rad/s]

// 读取前/后轴位置 (精确几何计算)
const double front_x = state.car_state_front.x;
const double rear_x  = state.car_state_rear.x;
```

---

## Task #17: INS 时间戳字段和单位文档 ✅

### HUAT_InsP2 关键字段

| 字段名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| `header.stamp` | `ros::Time` | s | ROS 时间戳（硬件时间或系统时间） |
| `Lat` / `Lon` | `double` | 度 (°) | WGS84 经纬度 |
| `Altitude` | `double` | m | 海拔高度 |
| `Vn` / `Ve` / `Vd` | `float` | m/s | NED 坐标系速度 (North/East/Down) |
| `Roll` / `Pitch` | `float` | 度 (°) | 横滚/俯仰角 |
| `Heading` | `float` | 度 (°) | 航向角 (0=North，顺时针) |
| `gyro_x/y/z` | `float` | rad/s | FRD 车体系角速度 |
| `acc_x/y/z` | `float` | m/s² | FRD 车体系加速度 |
| `Status` | `uint8` | - | INS 状态码 (≥2 = 有效定位) |
| `NSV1` / `NSV2` | `uint8` | - | 卫星数 (≥12 = 好信号) |
| `Age` | `uint8` | s | 差分龄期 (<5s = 好) |

### 时间戳防护规则

```
1. isZero() → 回退到 ros::Time::now()
2. stamp < last_stamp → 丢弃帧 (ins_stamp_rollback_count_++)
3. stamp > now + 1s → TF_future_stamp_count++，WARN日志
```

---

## Task #18: 规划模式切换状态机文档 ✅

### Mission 状态机 (MissionStateMachine)

```
                  ┌─────────────┐
    start ──────► │  IDLE/INIT  │
                  └──────┬──────┘
                         │ first valid path
                         ▼
                  ┌─────────────┐
                  │  MAP_BUILD  │ ◄── 建图中（SAFE_LAP）
                  └──────┬──────┘
                         │ loop_closed (闭环)
                         ▼
                  ┌─────────────┐
                  │  FAST_LAP   │ ◄── 全速圈（完整路径发布）
                  └──────┬──────┘
                         │ inter_times > required_laps
                         ▼
                  ┌─────────────┐
                  │   FINISH    │ ──► ros::shutdown()
                  └─────────────┘
```

### replan 字段语义 (Task #14 实现)

`HUAT_PathLimits.replan` 字段含义：

- `true` = 本帧路径与上一帧**不同**（checksum 变化 或 闭环状态切换）
- `false` = 路径与上一帧相同，下游可复用缓存
- 首帧始终为 `true`（`last_path_checksum_` 初始为 0.0）

控制层可用此字段优化：仅在 `replan=true` 时重新计算追踪点。

---

## Task #22: 配置文件注释模板 ✅

### 标准 YAML 参数注释格式

```yaml
# ── [模块名] ─────────────────────────────────────────────────────
# 描述: 一句话说明参数用途
# 单位: m / m/s / deg / - / bool
# 范围: [最小值, 最大值]  或  可选值: [val1, val2]
# 影响: 说明参数对系统行为的影响
param_name: 默认值
```

### 示例 (vehicle_geometry.yaml)

```yaml
# ── 车辆几何参数 ──────────────────────────────────────────────────
# 描述: IMU 到前轴中心的纵向偏移
# 单位: m (正值 = IMU 在前轴前方)
# 范围: [-2.0, 2.0]
front_to_imu_x: 0.8

# 描述: IMU 到后轴中心的纵向偏移
# 单位: m (负值 = 后轴在 IMU 后方)
rear_to_imu_x: -0.75

# 描述: 轴距
# 单位: m
wheelbase: 1.55
```
