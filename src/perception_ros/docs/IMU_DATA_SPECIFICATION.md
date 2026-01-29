# IMU 数据处理规范文档

本文档描述感知系统中 IMU/INS 数据的处理流程、坐标系定义、单位转换规则及点云运动畸变补偿的实现细节。

---

## 1. 数据流架构

```mermaid
graph LR
    A[INS5711DAA<br/>硬件设备] -->|原始数据| B[/INS/ASENSING_INS<br/>ins/ASENSING.msg]
    B -->|ins_bridge.py<br/>单位转换+坐标系转换| C[/pbox_pub/Ins<br/>HUAT_InsP2.msg]
    C --> D[imu_subscriber.cpp]
    D --> E[IMUData 结构体]
    E --> F[DistortionCompensator<br/>点云畸变补偿]
    E --> G[localization<br/>定位模块]
```

---

## 2. 坐标系定义

### 2.1 车体坐标系 (FRD - Front-Right-Down)

用于加速度、角速度等车体相关量。

```
        X (Front, 前)
         ↑
         │
         │
    Y ←──●──→ 
  (Left)  │  (Right, 右)
         │
         ↓
         Z (Down, 下)
```

| 轴 | 正方向 | 说明 |
|---|---|---|
| X | 车头前进方向 | 前向加速度为正 |
| Y | 车体右侧 | 右向加速度为正 |
| Z | 指向地面 | **向下为正**，静止时 az ≈ +9.78 m/s² |

### 2.2 导航坐标系 (NED - North-East-Down)

用于速度等导航相关量。

| 轴 | 正方向 | 说明 |
|---|---|---|
| N | 正北 | 北向速度为正 |
| E | 正东 | 东向速度为正 |
| D | 向下 | 下降速度为正 |

### 2.3 姿态角定义

| 角度 | 符号 | 正方向 | 参考 |
|---|---|---|---|
| Heading (航向角) | ψ | 顺时针 | 正北为 0°，正东为 90° |
| Pitch (俯仰角) | θ | 抬头为正 | 水平为 0° |
| Roll (横滚角) | φ | 右倾为正 | 水平为 0° |

> [!IMPORTANT]
> Heading 的参考系依赖于 INS_Status：
> - Status=1：以上电姿态初始化时为 0°
> - Status=2：以正北为 0°（组合导航正常）

---

## 3. 消息定义

### 3.1 原始消息: `ins/ASENSING.msg`

设备直接输出的原始数据，**未经单位转换**。

| 字段 | 单位 | 坐标系 | 备注 |
|---|---|---|---|
| `x/y/z_acc` | **g** | FLU (Up正) | 静止时 z_acc ≈ -1.0 |
| `x/y/z_angular_velocity` | **deg/s** | FLU | |
| `north/east_velocity` | m/s | NED | |
| `ground_velocity` | m/s | NED | 向下为正 |
| `roll/pitch/azimuth` | deg | | |
| `latitude/longitude` | deg | WGS84 | |
| `altitude` | m | | |

### 3.2 转换后消息: `autodrive_msgs/HUAT_InsP2.msg`

经过 `ins_bridge.py` 转换后的标准化消息。

| 字段 | 单位 | 坐标系 | 转换规则 |
|---|---|---|---|
| `acc_x/y` | **m/s²** | FRD | ×9.7883105 |
| `acc_z` | **m/s²** | FRD | **-1 × z_acc × 9.7883** (取反) |
| `gyro_x/y/z` | **rad/s** | FRD | ×π/180 |
| `Vn/Ve` | m/s | NED | 直接复制 |
| `Vd` | m/s | NED | 向下为正 |
| `Heading/Pitch/Roll` | deg | | 直接复制 |
| `Lat/Lon` | deg | WGS84 | |
| `Altitude` | m | | |

> [!CAUTION]
> `acc_z` 需要取反！原始 ASENSING 使用 FLU 坐标系（向上为正），而 HUAT_InsP2 使用 FRD（向下为正）。

---

## 4. 单位转换详解

### 4.1 转换常量

```cpp
constexpr double DEG2RAD = M_PI / 180.0;  // 0.01745329...
constexpr double G = 9.7883105;           // 当地重力加速度 (m/s²)
```

### 4.2 转换流程 (`ins_bridge.py`)

```python
# 角速度: deg/s -> rad/s
out.gyro_x = msg.x_angular_velocity * DEG2RAD
out.gyro_y = msg.y_angular_velocity * DEG2RAD
out.gyro_z = msg.z_angular_velocity * DEG2RAD

# 加速度: g -> m/s², 同时 FLU -> FRD (Z轴取反)
out.acc_x = msg.x_acc * G
out.acc_y = msg.y_acc * G
out.acc_z = -msg.z_acc * G  # 注意取反！
```

### 4.3 消费端使用 (`imu_subscriber.cpp`)

```cpp
// HUAT_InsP2 中已经是标准单位，直接使用
imu_data.acceleration.ax = imu_msg_ptr->acc_x;  // m/s²
imu_data.acceleration.ay = imu_msg_ptr->acc_y;  // m/s²
imu_data.acceleration.az = imu_msg_ptr->acc_z;  // m/s²

imu_data.angular_velocity.wx = imu_msg_ptr->gyro_x;  // rad/s
imu_data.angular_velocity.wy = imu_msg_ptr->gyro_y;  // rad/s
imu_data.angular_velocity.wz = imu_msg_ptr->gyro_z;  // rad/s
```

> [!WARNING]
> **不要在消费端重复转换！** HUAT_InsP2 消息中的值已经是最终单位。

---

## 5. INS 状态定义

| Status | 含义 | 有效数据 |
|---|---|---|
| 0 | NONE (未初始化) | 无 |
| 1 | 姿态初始化完成 | 姿态角、角速度、加速度 |
| 2 | 组合导航正常 | 全部数据有效 |

```cpp
// 数据有效性判断
imu_data.status.pos_valid = (Status == 2);      // 位置
imu_data.status.vel_valid = (Status >= 1);      // 速度
imu_data.status.att_valid = (Status >= 1);      // 姿态
imu_data.status.heading_valid = (Status == 2);  // 航向(北参考)
```

---

## 6. 点云运动畸变补偿

### 6.1 原理

LiDAR 扫描一圈需要约 100ms，期间车辆运动会导致点云畸变。利用 IMU 数据估计扫描期间的运动，对每个点进行补偿。

### 6.2 配置参数

```yaml
# config/lidar_cluster_example.yaml
distortion_compensation:
  enable: true
  imu_topic: "/pbox_pub/Ins"
  scan_period: 0.1  # 扫描周期 (秒)
```

### 6.3 补偿算法

```cpp
// 对每个点计算其采集时刻的车体位姿变化
// dt = 点的相对时间 (0 ~ scan_period)
// 使用角速度积分估计旋转
Eigen::AngleAxisd rotation(angular_velocity * dt, axis);
// 使用速度积分估计平移
Eigen::Vector3d translation = velocity * dt;
// 将点变换回扫描起始时刻的位置
point_corrected = rotation.inverse() * (point - translation);
```

---

## 7. 数据验证方法

### 7.1 静止状态检查

```bash
# 车辆静止时，检查 acc_z 应该约等于 +9.78
rostopic echo /pbox_pub/Ins/acc_z
# 期望输出: 9.7 ~ 9.9 (正值)
```

### 7.2 角速度检查

```bash
# 车辆静止时，角速度应该接近 0
rostopic echo /pbox_pub/Ins/gyro_z
# 期望输出: 接近 0 (< 0.05 rad/s)
```

### 7.3 速度检查

```bash
# 车辆前进时，Vn/Ve 应该有合理数值
rostopic echo /pbox_pub/Ins -n 1 | grep -E "V[ned]"
```

---

## 8. 常见问题排查

### Q1: `acc_z` 显示负值 (-9.78)

**原因**: `ins_bridge.py` 中 `acc_z` 未取反
**解决**: 修改为 `out.acc_z = -msg.z_acc * self.G`

### Q2: 角速度值过小 (约 0.01)

**原因**: 可能在消费端重复做了 `×DEG2RAD` 转换
**解决**: 确保 `imu_subscriber.cpp` 直接使用 `gyro_x/y/z`，不再转换

### Q3: 消息 md5sum 不匹配

**原因**: 修改 `.msg` 文件后未重新编译
**解决**: `catkin build --force-cmake && source devel/setup.bash`

### Q4: IMU buffer empty 警告

**原因**: IMU 消息未发布或时间戳不同步
**解决**: 
1. 检查 `ins_bridge` 节点是否运行
2. 确保 rosbag 使用 `--clock` 参数播放
3. 检查 `use_sim_time` 设置

---

## 9. 相关文件

| 文件 | 作用 |
|---|---|
| [`ins/msg/ASENSING.msg`](../../ins/msg/ASENSING.msg) | 原始 INS 消息定义 |
| [`autodrive_msgs/msg/HUAT_InsP2.msg`](../../autodrive_msgs/msg/HUAT_InsP2.msg) | 标准化 INS 消息定义 |
| [`ins/scripts/ins_bridge.py`](../../ins/scripts/ins_bridge.py) | 消息转换桥接节点 |
| [`perception_ros/src/imu_subscriber.cpp`](../src/imu_subscriber.cpp) | IMU 订阅器 |
| [`perception_ros/include/perception_ros/imu_subscriber.hpp`](../include/perception_ros/imu_subscriber.hpp) | IMU 订阅器头文件 |
| [`perception_core/include/perception_core/imu_data.hpp`](../../perception_core/include/perception_core/imu_data.hpp) | IMU 数据结构定义 |

---

## 10. 参考资料

- INS5711DAA 使用手册 V1.05
- ROS sensor_msgs/Imu 消息规范
- PCL 点云运动补偿算法

---

> 文档版本: 1.0  
> 最后更新: 2026-01-29  
> 维护者: HUAT Racing Team
