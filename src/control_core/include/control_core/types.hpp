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
  double theta{0.0};
  double v{0.0};
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
