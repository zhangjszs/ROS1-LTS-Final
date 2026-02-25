#include "control_core/skip_controller.hpp"

#include <algorithm>

namespace control_core
{

void SkipController::UpdateCarState(const CarState &state)
{
  ControllerBase::UpdateCarState(state);
  car_fangle_ = state.theta;
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
  int index_min = FindNearestIndex();
  tar_ = index_min;

  // B26: Bounds-safe access — clamp to second-to-last point
  const int last = static_cast<int>(path_coordinate_.size()) - 1;
  if (index_min >= last)
  {
    if (finish_signal_)
    {
      RequestStop();
      return steering_offset_;
    }
    index_min = std::max(last - 1, 0);
  }

  double alpha = std::atan2(path_coordinate_[index_min + 1].y - path_coordinate_[index_min].y,
                            path_coordinate_[index_min + 1].x - path_coordinate_[index_min].x) - car_fangle_;

  alpha = angle_range(alpha);

  double e_y = CountError(path_coordinate_[index_min].x, path_coordinate_[index_min].y, car_x_, car_y_, alpha);

  double delta = std::atan2(0.5 * e_y, car_veloc_ + 6) + alpha;

  delta = angle_pid(delta);

  return static_cast<int>(delta / M_PI * 180 * steering_ratio_) + steering_offset_;
}

int SkipController::ComputePedal()
{
  return ComputeDefaultPedal();
}

int SkipController::ComputeBrake()
{
  return ComputeDefaultBrake();
}

int SkipController::ComputeStatus()
{
  return ComputeDefaultStatus();
}

} // namespace control_core
