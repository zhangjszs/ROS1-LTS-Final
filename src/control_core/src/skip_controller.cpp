#include "control_core/skip_controller.hpp"

#include <limits>

namespace control_core
{

void SkipController::UpdateCarState(const CarState &state)
{
  ControllerBase::UpdateCarState(state);
  car_fangle_ = state.theta;
}

int SkipController::GetMinIndex()
{
  double min_distance = std::numeric_limits<double>::max();
  int index = 0;
  int min = 0;
  lookhead_ = angle_kv_ * car_veloc_ + angle_kl_;
  for (min = index = 0; index < static_cast<int>(path_coordinate_.size()); index++)
  {
    double tem_distance = distance_square(car_x_, car_y_, path_coordinate_[index].x, path_coordinate_[index].y);
    if (tem_distance < min_distance)
    {
      min_distance = tem_distance;
      min = index;
    }
  }
  tar_ = min;
  return min;
}

double SkipController::CountError(double x1, double y1, double x2, double y2, double heading) const
{
  double dx = x1 - x2;
  double dy = y1 - y2;
  double error = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
  return dy * std::cos(heading) - dx * std::sin(heading) > 0 ? -error : error;
}

int SkipController::ComputeSteering()
{
  int index_min = GetMinIndex();
  if (index_min >= static_cast<int>(path_coordinate_.size()) - 1 && finish_signal_)
  {
    RequestStop();
    return 110;
  }
  double alpha = std::atan2(path_coordinate_[index_min + 1].y - path_coordinate_[index_min].y,
                            path_coordinate_[index_min + 1].x - path_coordinate_[index_min].x) - car_fangle_;

  alpha = angle_range(alpha);

  double e_y = CountError(path_coordinate_[index_min].x, path_coordinate_[index_min].y, car_x_, car_y_, alpha);

  double delta = std::atan2(0.5 * e_y, car_veloc_ + 6) + alpha;

  delta = angle_pid(delta);

  return static_cast<int>(delta / M_PI * 180 * 3.73) + 110;
}

int SkipController::ComputePedal()
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

int SkipController::ComputeBrake()
{
  return 0;
}

int SkipController::ComputeStatus()
{
  return 2;
}

} // namespace control_core
