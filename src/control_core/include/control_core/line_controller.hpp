#ifndef CONTROL_CORE_LINE_CONTROLLER_HPP_
#define CONTROL_CORE_LINE_CONTROLLER_HPP_

#include "control_core/controller_base.hpp"

namespace control_core
{

class LineController : public ControllerBase
{
public:
  LineController() = default;

protected:
  int ComputeSteering() override;
  int ComputePedal() override;
  int ComputeBrake() override;
  int ComputeStatus() override;

private:
  int GetTargetIndex();
};

} // namespace control_core

#endif // CONTROL_CORE_LINE_CONTROLLER_HPP_
