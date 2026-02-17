#include "control_core/controller_base.hpp"

#include <limits>

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

  steering_ratio_ = params.steering_ratio;
  steering_offset_ = params.steering_offset;

  // FSSIM风格参数
  cg_to_front_ = params.cg_to_front;
  cg_to_rear_ = params.cg_to_rear;
  mass_ = params.mass;
  enable_slip_compensation_ = params.enable_slip_compensation;
  slip_gain_ = params.slip_gain;
  min_lookahead_ = params.min_lookahead;
  max_lookahead_ = params.max_lookahead;

  // B22: critical parameter range validation
  if (car_length_ <= 0.0) { car_length_ = 1.55; }
  if (steering_delta_max_ <= 0.0) { steering_delta_max_ = 0.5; }
  if (min_lookahead_ <= 0.0) { min_lookahead_ = 0.5; }
  if (max_lookahead_ < min_lookahead_) { max_lookahead_ = min_lookahead_ + 1.0; }
}

void ControllerBase::UpdateCarState(const CarState &state)
{
  car_x_ = state.x;
  car_y_ = state.y;
  car_theta_ = state.theta;
  car_veloc_ = state.v;

  // FSSIM风格扩展状态
  car_vy_ = state.vy;
  car_yaw_rate_ = state.yaw_rate;
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

int ControllerBase::GetTargetIndex()
{
  double min_distance = std::numeric_limits<double>::max();
  double diff_distance = 0;
  double tem_distance;
  int index = 0;
  int min = 0;

  lookhead_ = computeAdaptiveLookahead();

  for (min = index = 0; index < static_cast<int>(path_coordinate_.size()); index++)
  {
    double tmp_distance = distance_square(car_x_, car_y_, path_coordinate_[index].x, path_coordinate_[index].y);
    if (tmp_distance < min_distance)
    {
      min_distance = tmp_distance;
      min = index;
    }
  }

  for (index = min + 1, diff_distance = 0.0; index < static_cast<int>(path_coordinate_.size()); index++)
  {
    tem_distance = std::sqrt(distance_square(path_coordinate_[index - 1].x, path_coordinate_[index - 1].y,
                                             path_coordinate_[index].x, path_coordinate_[index].y));
    diff_distance += tem_distance;
    if (diff_distance + tem_distance > lookhead_)
      break;
  }
  if (std::abs(lookhead_ - diff_distance - tem_distance) < std::abs(lookhead_ - diff_distance))
    return index;
  else
    return index - 1;
}

double ControllerBase::angle_range(double alpha) const
{
  if (!std::isfinite(alpha)) { return 0.0; }
  return std::remainder(alpha, 2.0 * M_PI);
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

double ControllerBase::computeSlipAngle() const
{
  // FSSIM风格滑移角计算
  // beta = atan(vy / vx)
  // 在低速时避免除零
  const double min_vx = 0.5;
  double vx = std::max(car_veloc_, min_vx);
  return std::atan2(car_vy_, vx);
}

double ControllerBase::compensateSlipAngle(double delta) const
{
  if (!enable_slip_compensation_ || car_veloc_ < 1.0)
  {
    return delta;
  }

  // FSSIM风格滑移角补偿
  // 补偿公式: delta_comp = delta - slip_gain * beta
  double beta = computeSlipAngle();
  double delta_comp = delta - slip_gain_ * beta;

  // 限幅
  if (delta_comp > steering_delta_max_)
    delta_comp = steering_delta_max_;
  else if (delta_comp < -steering_delta_max_)
    delta_comp = -steering_delta_max_;

  return delta_comp;
}

double ControllerBase::computeAdaptiveLookahead() const
{
  // FSSIM风格速度自适应前视距离
  // lookahead = kv * v + kl
  // 但限制在 [min_lookahead, max_lookahead] 范围内
  double lookahead = angle_kv_ * car_veloc_ + angle_kl_;
  if (lookahead < min_lookahead_)
    lookahead = min_lookahead_;
  else if (lookahead > max_lookahead_)
    lookahead = max_lookahead_;
  return lookahead;
}

int ControllerBase::ComputeSteeringWithLookahead(int target_index)
{
  if (target_index >= static_cast<int>(path_coordinate_.size()) - 1 && finish_signal_)
  {
    RequestStop();
    return steering_offset_;
  }

  double dx = path_coordinate_[target_index].x - car_x_;
  double dy = path_coordinate_[target_index].y - car_y_;
  double goalX = std::cos(car_theta_) * dx + std::sin(car_theta_) * dy;
  double goalY = -std::sin(car_theta_) * dx + std::cos(car_theta_) * dy;

  double alpha = std::atan2(goalY, goalX);
  alpha = angle_range(alpha);

  double delta = std::atan2(2 * car_length_ * std::sin(alpha) / lookhead_, 1.0);

  delta = compensateSlipAngle(delta);

  delta = angle_pid(delta);

  return static_cast<int>(delta / M_PI * 180 * steering_ratio_) + steering_offset_;
}

int ControllerBase::ComputeDefaultPedal()
{
  double error = default_target_speed_ - car_veloc_;
  double accel;
  veloc_integra_ += error;
  accel = default_pedal_kp_ * error + default_pedal_ki_ * veloc_integra_;

  if (car_veloc_ > default_high_speed_threshold_)
    accel = default_pedal_cap_;

  if (accel > default_pedal_max_)
    accel = default_pedal_cap_;

  if (car_veloc_ <= default_min_speed_threshold_)
    accel = default_pedal_cap_;
  return static_cast<int>(accel);
}

int ControllerBase::ComputeDefaultBrake()
{
  return 0;
}

int ControllerBase::ComputeDefaultStatus()
{
  return 2;
}

} // namespace control_core
