#ifndef CONTROL_CORE_HIGH_CONTROLLER_HPP_
#define CONTROL_CORE_HIGH_CONTROLLER_HPP_

#include "control_core/line_controller.hpp"

namespace control_core {

/**
 * @brief High-speed tracking controller.
 *
 * Currently shares the same implementation as LineController. It exists as a
 * distinct type so that future high-speed-specific tuning (e.g. longer
 * lookahead, different curvature feedforward gain) can be added without
 * changing call sites.
 */
class HighController : public LineController {
 public:
  HighController() = default;
};

}  // namespace control_core

#endif  // CONTROL_CORE_HIGH_CONTROLLER_HPP_
