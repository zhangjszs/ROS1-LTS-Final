#ifndef CONTROL_CORE_TEST_CONTROLLER_HPP_
#define CONTROL_CORE_TEST_CONTROLLER_HPP_

#include "control_core/controller_base.hpp"

namespace control_core
{

class TestController : public ControllerBase
{
public:
  TestController() = default;

protected:
  int ComputeSteering() override;
  int ComputePedal() override;
  int ComputeBrake() override;
  int ComputeStatus() override;
};

} // namespace control_core

#endif // CONTROL_CORE_TEST_CONTROLLER_HPP_
