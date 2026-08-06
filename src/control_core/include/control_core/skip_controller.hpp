#ifndef CONTROL_CORE_SKIP_CONTROLLER_HPP_
#define CONTROL_CORE_SKIP_CONTROLLER_HPP_

#include "control_core/controller_base.hpp"

namespace control_core {

class SkipController : public ControllerBase {
 public:
  SkipController() = default;
  void UpdateCarState(const CarState& state) override;

  /**
   * @brief Compute signed cross-track error relative to a path segment.
   * @param x1 Path point x
   * @param y1 Path point y
   * @param x2 Vehicle x
   * @param y2 Vehicle y
   * @param heading Absolute path tangent angle (radians)
   * @return Signed lateral error (positive = vehicle left of path direction)
   */
  double CountError(double x1, double y1, double x2, double y2, double heading) const;

 protected:
  int ComputeSteering() override;
  int ComputePedal() override;
  int ComputeBrake() override;
  int ComputeStatus() override;
};

}  // namespace control_core

#endif  // CONTROL_CORE_SKIP_CONTROLLER_HPP_
