#include "control_core/ebs_controller.hpp"

namespace control_core {

int EbsController::ComputeSteering() {
  return 110;
}

int EbsController::ComputePedal() {
  return 0;
}

int EbsController::ComputeBrake() {
  return 100;
}

int EbsController::ComputeStatus() {
  return 3;
}

}  // namespace control_core
