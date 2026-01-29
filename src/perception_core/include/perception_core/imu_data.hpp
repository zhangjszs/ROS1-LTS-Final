/*
 * @Description: IMU数据结构定义
 * @Author: Modified based on original code
 * @Date: 2026-01-29
 * 
 * 坐标系说明:
 * - 速度: NED坐标系 (North-East-Down)
 * - 加速度/角速度: FRD车体坐标系 (Front-Right-Down)
 * - 姿态角: 欧拉角 (Roll-Pitch-Yaw)
 * 
 * 单位:
 * - 速度: m/s
 * - 加速度: m/s²
 * - 角速度: rad/s
 * - 姿态角: 度 (degree)
 * - 时间: 秒 (s)
 */

#ifndef MULTI_SENSOR_FUSION_IMU_DATA_HPP_
#define MULTI_SENSOR_FUSION_IMU_DATA_HPP_

class IMUData {
public:
  /**
   * @brief 速度结构 (NED坐标系)
   * North-East-Down 坐标系，常用于导航系统
   */
  struct Velocity {
    double vn = 0.0;  ///< 北向速度 (m/s)
    double ve = 0.0;  ///< 东向速度 (m/s)
    double vd = 0.0;  ///< 地向速度，向下为正 (m/s)
  };

  /**
   * @brief 加速度结构 (FRD车体坐标系)
   * Front-Right-Down 坐标系，与 INS5711DAA 设备输出一致
   */
  struct Acceleration {
    double ax = 0.0;  ///< X轴加速度，前向为正 (m/s²)
    double ay = 0.0;  ///< Y轴加速度，右向为正 (m/s²)
    double az = 0.0;  ///< Z轴加速度，下向为正 (m/s²)
  };

  /**
   * @brief 角速度结构 (FRD车体坐标系)
   */
  struct AngularVelocity {
    double wx = 0.0;  ///< 绕X轴角速度 (Roll rate, rad/s)
    double wy = 0.0;  ///< 绕Y轴角速度 (Pitch rate, rad/s)
    double wz = 0.0;  ///< 绕Z轴角速度 (Yaw rate, rad/s)
  };

  /**
   * @brief 姿态角结构 (欧拉角)
   * 注意: Heading 的参考系依赖于 INS_Status
   *       - INS_Status=1: 以上电姿态初始化时为 0°
   *       - INS_Status=2: 以正北为 0°，顺时针为正
   */
  struct Orientation {
    double roll = 0.0;     ///< 横滚角，右倾为正 (度)
    double pitch = 0.0;    ///< 俯仰角，抬头为正 (度)
    double heading = 0.0;  ///< 航向角，北0°顺时针 (度)
  };

  /**
   * @brief INS 状态信息
   * 用于判断数据有效性
   */
  struct InsStatus {
    uint8_t overall = 0;       ///< 总体状态: 0=NONE, 1=姿态初始化, 2=组合导航
    bool pos_valid = false;    ///< 位置有效标志
    bool vel_valid = false;    ///< 速度有效标志
    bool att_valid = false;    ///< 姿态有效标志
    bool heading_valid = false;///< 航向有效标志
    uint8_t num_sv = 0;        ///< 卫星数量
    double diff_age = 0.0;     ///< 差分龄期 (秒)
  };

  // === 公共成员变量 ===
  double time = 0.0;                     ///< 时间戳 (秒)
  Velocity velocity;                     ///< 速度 (m/s, NED)
  Acceleration acceleration;             ///< 加速度 (m/s², FRD)
  AngularVelocity angular_velocity;      ///< 角速度 (rad/s, FRD)
  Orientation orientation;               ///< 姿态角 (度)
  InsStatus status;                      ///< INS 状态信息
};

#endif  // MULTI_SENSOR_FUSION_IMU_DATA_HPP_
