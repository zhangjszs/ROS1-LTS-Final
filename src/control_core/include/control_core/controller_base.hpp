#ifndef CONTROL_CORE_CONTROLLER_BASE_HPP_
#define CONTROL_CORE_CONTROLLER_BASE_HPP_

#include <cmath>
#include <vector>

#include "control_core/types.hpp"

namespace control_core
{

class ControllerBase
{
public:
  virtual ~ControllerBase() = default;

  void SetParams(const ControlParams &params);
  virtual void UpdateCarState(const CarState &state);
  void UpdatePath(const std::vector<Position> &path);
  void SetFinishSignal(bool finish_signal);

  ControlOutput ComputeOutput();
  void Tick();

  bool HasPath() const { return !path_coordinate_.empty(); }

protected:
  virtual int ComputeSteering() = 0;
  virtual int ComputePedal() = 0;
  virtual int ComputeBrake() = 0;
  virtual int ComputeStatus() = 0;

  double distance_square(double x1, double y1, double x2, double y2) const;
  double angle_range(double alpha) const;
  double angle_pid(double delta);

  void RequestStop();

  std::vector<Position> path_coordinate_{};

  double car_x_{0.0};
  double car_y_{0.0};
  double car_theta_{0.0};
  double car_veloc_{0.0};
  double car_fangle_{0.0};

  double lookhead_{0.0};
  double angle_kv_{0.0};
  double angle_kl_{2.0};
  double angle_kp_{1.0};
  double angle_ki_{0.0};
  double angle_kd_{0.0};
  double steering_delta_max_{0.5};
  double car_length_{1.55};

  double angle_integra_{0.0};
  double veloc_integra_{0.0};

  int tar_{0};
  int now_{0};

  bool finish_signal_{false};
  bool stop_requested_{false};
};

} // namespace control_core

#endif // CONTROL_CORE_CONTROLLER_BASE_HPP_
