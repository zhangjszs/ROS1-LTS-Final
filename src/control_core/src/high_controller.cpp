#include "control_core/high_controller.hpp"

namespace control_core
{

int HighController::ComputeSteering()
{
  int index = GetTargetIndex();
  return ComputeSteeringWithLookahead(index);
}

int HighController::ComputePedal()
{
  return ComputeDefaultPedal();
}

int HighController::ComputeBrake()
{
  return ComputeDefaultBrake();
}

int HighController::ComputeStatus()
{
  return ComputeDefaultStatus();
}

} // namespace control_core
