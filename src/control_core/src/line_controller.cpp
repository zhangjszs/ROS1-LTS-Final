#include "control_core/line_controller.hpp"

#include <limits>

namespace control_core
{

int LineController::GetTargetIndex()
{
  double min_distance = std::numeric_limits<double>::max();
  double diff_distance = 0;
  double tem_distance;
  int index = 0;
  int min = 0;

  // FSSIM风格：使用自适应前视距离
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

int LineController::ComputeSteering()
{
  int index = GetTargetIndex();
  tar_ = index;
  if (index >= static_cast<int>(path_coordinate_.size()) - 1 && finish_signal_)
  {
    RequestStop();
    return 110;
  }

  double dx = path_coordinate_[index].x - car_x_;
  double dy = path_coordinate_[index].y - car_y_;
  double goalX = std::cos(car_theta_) * dx + std::sin(car_theta_) * dy;
  double goalY = -std::sin(car_theta_) * dx + std::cos(car_theta_) * dy;
  double alpha = std::atan2(goalY, goalX);

  alpha = angle_range(alpha);

  // 纯追踪转向角计算
  double delta = std::atan2(2 * car_length_ * std::sin(alpha) / lookhead_, 1.0);

  // FSSIM风格：滑移角补偿
  delta = compensateSlipAngle(delta);

  delta = angle_pid(delta);

  return static_cast<int>(delta / M_PI * 180 * 3.73) + 110;
}

int LineController::ComputePedal()
{
  double target = 4.0;
  double error = target - car_veloc_;
  double accel;
  veloc_integra_ += error;
  accel = 0.5 * error + 0.1 * veloc_integra_;

  if (car_veloc_ > 2)
    accel = 5;

  if (accel > 30)
    accel = 5;

  if (car_veloc_ <= 0.3)
    accel = 5;
  return static_cast<int>(accel);
}

int LineController::ComputeBrake()
{
  return 0;
}

int LineController::ComputeStatus()
{
  return 2;
}

} // namespace control_core
