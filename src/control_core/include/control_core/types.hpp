#ifndef CONTROL_CORE_TYPES_HPP_
#define CONTROL_CORE_TYPES_HPP_

#include <vector>

namespace control_core
{

struct Position
{
  double x{0.0};
  double y{0.0};
};

struct CarState
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};    // 偏航角 [rad]
  double v{0.0};        // 纵向速度 [m/s]

  // FSSIM风格扩展状态
  double vy{0.0};       // 横向速度 [m/s]
  double yaw_rate{0.0}; // 偏航角速度 [rad/s]
  double ax{0.0};       // 纵向加速度 [m/s^2]
  double ay{0.0};       // 横向加速度 [m/s^2]
};

struct ControlParams
{
  double angle_kp{1.0};
  double angle_ki{0.0};
  double angle_kd{0.0};
  double angle_kv{0.0};
  double angle_kl{2.0};
  double steering_delta_max{0.5};
  double car_length{1.55};

  // FSSIM风格车辆参数
  double cg_to_front{0.77};     // 重心到前轴距离 [m]
  double cg_to_rear{0.78};      // 重心到后轴距离 [m]
  double mass{190.0};           // 车辆质量 [kg]

  // 滑移角补偿参数
  bool enable_slip_compensation{true};
  double slip_gain{0.5};        // 滑移角补偿增益

  // 速度自适应参数
  double min_lookahead{2.0};    // 最小前视距离 [m]
  double max_lookahead{10.0};   // 最大前视距离 [m]
};

struct ControlOutput
{
  int steering{110};
  int pedal_ratio{0};
  int brake_force{0};
  int racing_status{2};
  bool stop_requested{false};
};

} // namespace control_core

#endif // CONTROL_CORE_TYPES_HPP_
