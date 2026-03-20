#ifndef CONTROL_CORE_EBS_CONTROLLER_HPP_
#define CONTROL_CORE_EBS_CONTROLLER_HPP_

#include "control_core/controller_base.hpp"

namespace control_core {

class EbsController : public ControllerBase {
 public:
  EbsController() = default;

 protected:
  int ComputeSteering() override;
  int ComputePedal() override;
  int ComputeBrake() override;
  int ComputeStatus() override;
};

}  // namespace control_core

#endif  // CONTROL_CORE_EBS_CONTROLLER_HPP_
