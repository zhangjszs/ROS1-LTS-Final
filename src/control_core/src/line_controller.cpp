#include "control_core/line_controller.hpp"

namespace control_core
{

int LineController::ComputeSteering()
{
  int index = GetTargetIndex();
  return ComputeSteeringWithLookahead(index);
}

int LineController::ComputePedal()
{
  return ComputeDefaultPedal();
}

int LineController::ComputeBrake()
{
  return ComputeDefaultBrake();
}

int LineController::ComputeStatus()
{
  return ComputeDefaultStatus();
}

} // namespace control_core
