#include "control_core/controller_base.hpp"

namespace control_core
{

void ControllerBase::SetParams(const ControlParams &params)
{
  angle_kp_ = params.angle_kp;
  angle_ki_ = params.angle_ki;
  angle_kd_ = params.angle_kd;
  angle_kv_ = params.angle_kv;
  angle_kl_ = params.angle_kl;
  steering_delta_max_ = params.steering_delta_max;
  car_length_ = params.car_length;
}

void ControllerBase::UpdateCarState(const CarState &state)
{
  car_x_ = state.x;
  car_y_ = state.y;
  car_theta_ = state.theta;
  car_veloc_ = state.v;
}

void ControllerBase::UpdatePath(const std::vector<Position> &path)
{
  path_coordinate_ = path;
}

void ControllerBase::SetFinishSignal(bool finish_signal)
{
  finish_signal_ = finish_signal;
}

ControlOutput ControllerBase::ComputeOutput()
{
  stop_requested_ = false;
  ControlOutput output;
  output.steering = ComputeSteering();
  output.pedal_ratio = ComputePedal();
  output.brake_force = ComputeBrake();
  output.racing_status = ComputeStatus();
  output.stop_requested = stop_requested_;
  return output;
}

void ControllerBase::Tick()
{
  ++now_;
}

double ControllerBase::distance_square(double x1, double y1, double x2, double y2) const
{
  return std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
}

double ControllerBase::angle_range(double alpha) const
{
  return (alpha > M_PI) ? (alpha - 2 * M_PI) : (alpha < -M_PI) ? (alpha + 2 * M_PI)
                                                               : alpha;
}

double ControllerBase::angle_pid(double delta)
{
  double error = delta - car_fangle_;

  double differ = error - last_angle_error_;
  last_angle_error_ = error;

  if (std::abs(error) <= steering_delta_max_)
  {
    angle_integra_ += error;
  }
  else
  {
    angle_integra_ = 0.0;
  }

  double output = angle_kp_ * error + angle_ki_ * angle_integra_ + angle_kd_ * differ + car_fangle_;

  if (output > steering_delta_max_)
    output = steering_delta_max_;
  else if (output < -steering_delta_max_)
    output = -steering_delta_max_;

  car_fangle_ = output;
  return output;
}

void ControllerBase::RequestStop()
{
  stop_requested_ = true;
}

} // namespace control_core
