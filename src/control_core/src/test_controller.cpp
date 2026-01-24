#include "control_core/test_controller.hpp"

namespace control_core
{

int TestController::ComputeSteering()
{
  if (now_ > 350)
    return 110;
  else
    return 110 + static_cast<int>(40 * (std::sin((now_ % 60) * 6.0 * M_PI / 180.0)));
}

int TestController::ComputePedal()
{
  if (now_ > 350)
    return 0;
  else
    return 10;
}

int TestController::ComputeBrake()
{
  if (now_ > 350)
    return 80;
  else
    return 0;
}

int TestController::ComputeStatus()
{
  if (now_ > 350)
    return 4;
  else
    return 2;
}

} // namespace control_core
