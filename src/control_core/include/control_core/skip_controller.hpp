#ifndef CONTROL_CORE_SKIP_CONTROLLER_HPP_
#define CONTROL_CORE_SKIP_CONTROLLER_HPP_

#include "control_core/controller_base.hpp"

namespace control_core
{

class SkipController : public ControllerBase
{
public:
  SkipController() = default;
  void UpdateCarState(const CarState &state) override;

protected:
  int ComputeSteering() override;
  int ComputePedal() override;
  int ComputeBrake() override;
  int ComputeStatus() override;

private:
  int GetMinIndex();
  double CountError(double x1, double y1, double x2, double y2, double heading) const;
};

} // namespace control_core

#endif // CONTROL_CORE_SKIP_CONTROLLER_HPP_
