#include "control_core/controller_base.hpp"

#include <limits>

namespace control_core {

void ControllerBase::SetParams(const ControlParams& params) {
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
  curvature_ff_gain_ = params.curvature_ff_gain;
  brake_kp_ = params.brake_kp;
  brake_max_ = params.brake_max;
  brake_speed_margin_ = params.brake_speed_margin;

  // B22: critical parameter range validation
  if (car_length_ <= 0.0) {
    car_length_ = 1.55;
  }
  if (steering_delta_max_ <= 0.0) {
    steering_delta_max_ = 0.5;
  }
  if (min_lookahead_ <= 0.0) {
    min_lookahead_ = 0.5;
  }
  if (max_lookahead_ < min_lookahead_) {
    max_lookahead_ = min_lookahead_ + 1.0;
  }

  // M7: Validate safety-critical PID and brake parameters
  if (angle_kp_ < 0.0) {
    angle_kp_ = 0.0;
  }
  if (angle_ki_ < 0.0) {
    angle_ki_ = 0.0;
  }
  if (angle_kd_ < 0.0) {
    angle_kd_ = 0.0;
  }
  if (brake_kp_ < 0.0) {
    brake_kp_ = 0.0;
  }
  if (brake_max_ < 0.0) {
    brake_max_ = 0.0;
  }

  // M8: Clamp curvature feedforward gain to reasonable upper bound
  const double kMaxCurvatureFfGain = 5.0;
  if (curvature_ff_gain_ > kMaxCurvatureFfGain) {
    curvature_ff_gain_ = kMaxCurvatureFfGain;
  }
}

void ControllerBase::UpdateCarState(const CarState& state) {
  if (!std::isfinite(state.x) || !std::isfinite(state.y) || !std::isfinite(state.theta) ||
      !std::isfinite(state.v) || !std::isfinite(state.vy) || !std::isfinite(state.yaw_rate)) {
    return;
  }

  car_x_ = state.x;
  car_y_ = state.y;
  car_theta_ = state.theta;
  car_veloc_ = state.v;

  // FSSIM风格扩展状态
  car_vy_ = state.vy;
  car_yaw_rate_ = state.yaw_rate;
}

void ControllerBase::UpdatePath(const std::vector<Position>& path) {
  path_coordinate_ = path;
  // Clear stale data if path length changed
  if (target_speeds_.size() != path.size()) {
    target_speeds_.clear();
  }
  if (curvatures_.size() != path.size()) {
    curvatures_.clear();
  }
}

void ControllerBase::UpdateTargetSpeeds(const std::vector<double>& speeds) {
  target_speeds_ = speeds;
}

void ControllerBase::UpdateCurvatures(const std::vector<double>& curvatures) {
  curvatures_ = curvatures;
}

void ControllerBase::SetFinishSignal(bool finish_signal) {
  finish_signal_ = finish_signal;
}

ControlOutput ControllerBase::ComputeOutput() {
  stop_requested_ = false;
  ControlOutput output;
  output.steering = ComputeSteering();
  output.pedal_ratio = ComputePedal();
  output.brake_force = ComputeBrake();
  output.racing_status = ComputeStatus();
  output.stop_requested = stop_requested_;
  return output;
}

void ControllerBase::Tick() {
  ++now_;
}

double ControllerBase::distance_square(double x1, double y1, double x2, double y2) const {
  double dx = x1 - x2;
  double dy = y1 - y2;
  return dx * dx + dy * dy;
}

int ControllerBase::FindNearestIndex() const {
  double min_distance = std::numeric_limits<double>::max();
  int nearest = 0;
  for (int i = 0; i < static_cast<int>(path_coordinate_.size()); ++i) {
    double d = distance_square(car_x_, car_y_, path_coordinate_[i].x, path_coordinate_[i].y);
    if (d < min_distance) {
      min_distance = d;
      nearest = i;
    }
  }
  return nearest;
}

int ControllerBase::GetTargetIndex() {
  // H1: Early-return for paths too short to compute a target index
  if (path_coordinate_.size() < 2) {
    return 0;
  }

  double diff_distance = 0;
  double tem_distance = 0.0;  // H1: initialize to avoid UB on first use
  int index = 0;

  lookhead_ = computeAdaptiveLookahead();

  int min = FindNearestIndex();

  for (index = min + 1, diff_distance = 0.0; index < static_cast<int>(path_coordinate_.size());
       index++) {
    tem_distance =
        std::sqrt(distance_square(path_coordinate_[index - 1].x, path_coordinate_[index - 1].y,
                                  path_coordinate_[index].x, path_coordinate_[index].y));
    diff_distance += tem_distance;
    if (diff_distance > lookhead_)
      break;
  }
  // Choose the index whose cumulative distance is closest to lookahead
  if (std::abs(lookhead_ - diff_distance) < std::abs(lookhead_ - (diff_distance - tem_distance)))
    return index;
  else
    return index - 1;
}

double ControllerBase::angle_range(double alpha) const {
  if (!std::isfinite(alpha)) {
    return 0.0;
  }
  return std::remainder(alpha, 2.0 * M_PI);
}

double ControllerBase::angle_pid(double delta) {
  double error = delta - car_fangle_;

  double differ = error - last_angle_error_;
  last_angle_error_ = error;

  if (std::abs(error) <= steering_delta_max_) {
    angle_integra_ += error;
    const double max_angle_integra = steering_delta_max_ / std::max(angle_ki_, 1e-6);
    angle_integra_ = std::max(-max_angle_integra, std::min(angle_integra_, max_angle_integra));
  } else {
    // Decay accumulated integral when error is large to prevent oscillation on recovery
    angle_integra_ *= 0.9;
  }

  double output = angle_kp_ * error + angle_ki_ * angle_integra_ + angle_kd_ * differ + car_fangle_;

  if (output > steering_delta_max_)
    output = steering_delta_max_;
  else if (output < -steering_delta_max_)
    output = -steering_delta_max_;

  // Rate limiting: constrain the change in output per cycle
  // This prevents instantaneous steering jumps when error sign flips
  double delta_output = output - last_steering_output_;
  double max_delta = steering_delta_max_ * 0.5;  // Allow 50% of max per cycle
  if (delta_output > max_delta) {
    output = last_steering_output_ + max_delta;
  } else if (delta_output < -max_delta) {
    output = last_steering_output_ - max_delta;
  }
  last_steering_output_ = output;

  car_fangle_ = output;
  return output;
}

void ControllerBase::RequestStop() {
  stop_requested_ = true;
}

double ControllerBase::computeSlipAngle() const {
  // FSSIM风格滑移角计算
  // beta = atan(vy / vx)
  // 在低速时避免除零
  const double min_vx = 0.5;
  double vx = std::max(car_veloc_, min_vx);
  return std::atan2(car_vy_, vx);
}

double ControllerBase::compensateSlipAngle(double delta) const {
  if (!enable_slip_compensation_ || car_veloc_ < 1.0) {
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

double ControllerBase::computeAdaptiveLookahead() const {
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

int ControllerBase::ComputeSteeringWithLookahead(int target_index) {
  if (target_index < 0) {
    return steering_offset_;
  }

  // Allow target_index == size() to trigger finish signal (one past last point)
  if (target_index >= static_cast<int>(path_coordinate_.size()) - 1 && finish_signal_) {
    RequestStop();
    return steering_offset_;
  }

  if (target_index >= static_cast<int>(path_coordinate_.size())) {
    return steering_offset_;
  }

  double dx = path_coordinate_[target_index].x - car_x_;
  double dy = path_coordinate_[target_index].y - car_y_;
  double goalX = std::cos(car_theta_) * dx + std::sin(car_theta_) * dy;
  double goalY = -std::sin(car_theta_) * dx + std::cos(car_theta_) * dy;

  double alpha = std::atan2(goalY, goalX);
  alpha = angle_range(alpha);

  double delta = std::atan2(2 * car_length_ * std::sin(alpha) / lookhead_, 1.0);

  // Curvature feedforward: delta_ff = atan(L * kappa)
  if (curvature_ff_gain_ > 0.0 && !curvatures_.empty()) {
    double kappa = GetCurvatureAt(target_index);
    double delta_ff = std::atan(car_length_ * kappa);
    delta += curvature_ff_gain_ * delta_ff;
  }

  delta = compensateSlipAngle(delta);

  delta = angle_pid(delta);

  return static_cast<int>(delta / M_PI * 180 * steering_ratio_) + steering_offset_;
}

double ControllerBase::GetTargetSpeedAt(int index) const {
  if (target_speeds_.empty() || index < 0 || index >= static_cast<int>(target_speeds_.size())) {
    return default_target_speed_;
  }
  double v = target_speeds_[index];
  return (v > 0.0) ? v : default_target_speed_;
}

double ControllerBase::GetCurvatureAt(int index) const {
  if (curvatures_.empty() || index < 0 || index >= static_cast<int>(curvatures_.size())) {
    return 0.0;
  }
  return curvatures_[index];
}

int ControllerBase::ComputeDefaultPedal() {
  // Look up per-point target speed from planning layer
  double target_speed = default_target_speed_;
  if (!path_coordinate_.empty() && !target_speeds_.empty()) {
    int nearest = FindNearestIndex();
    target_speed = GetTargetSpeedAt(nearest);
  }

  double error = target_speed - car_veloc_;
  double accel;
  veloc_integra_ += error;
  const double max_integra = default_pedal_max_ / std::max(default_pedal_ki_, 1e-6);
  veloc_integra_ = std::max(-max_integra, std::min(veloc_integra_, max_integra));
  accel = default_pedal_kp_ * error + default_pedal_ki_ * veloc_integra_;

  if (car_veloc_ > default_high_speed_threshold_ && car_veloc_ > target_speed)
    accel = default_pedal_cap_;

  if (accel > default_pedal_max_)
    accel = default_pedal_max_;

  if (car_veloc_ <= default_min_speed_threshold_ && target_speed > 0.0)
    accel = default_pedal_cap_;
  return static_cast<int>(accel);
}

int ControllerBase::ComputeDefaultBrake() {
  // M4: If mission is complete, apply minimum braking to avoid coasting
  if (finish_signal_ && car_veloc_ > 0.1) {
    return static_cast<int>(brake_max_ * 0.3);  // 30% max brake as minimum stop brake
  }

  if (path_coordinate_.empty() || target_speeds_.empty()) {
    return 0;
  }

  int nearest = FindNearestIndex();
  double target_speed = GetTargetSpeedAt(nearest);
  double overspeed = car_veloc_ - target_speed;

  if (overspeed > brake_speed_margin_) {
    // Proportional braking: harder brake for larger overspeed
    double brake = brake_kp_ * (overspeed - brake_speed_margin_);
    if (brake > brake_max_)
      brake = brake_max_;
    return static_cast<int>(brake);
  }

  return 0;
}

int ControllerBase::ComputeDefaultStatus() {
  return 2;
}

}  // namespace control_core
