#ifndef CONTROL_CORE_CONTROLLER_BASE_HPP_
#define CONTROL_CORE_CONTROLLER_BASE_HPP_

#include "control_core/types.hpp"

#include <cmath>
#include <vector>

namespace control_core {

class ControllerBase {
 public:
  virtual ~ControllerBase() = default;

  void SetParams(const ControlParams& params);
  virtual void UpdateCarState(const CarState& state);
  void UpdatePath(const std::vector<Position>& path);
  void UpdateTargetSpeeds(const std::vector<double>& speeds);
  void UpdateCurvatures(const std::vector<double>& curvatures);
  void SetFinishSignal(bool finish_signal);

  ControlOutput ComputeOutput();
  void Tick();

  bool HasPath() const { return !path_coordinate_.empty(); }

 protected:
  virtual int ComputeSteering() = 0;
  virtual int ComputePedal() = 0;
  virtual int ComputeBrake() = 0;
  virtual int ComputeStatus() = 0;

  int FindNearestIndex() const;
  int GetTargetIndex();
  double distance_square(double x1, double y1, double x2, double y2) const;
  double angle_range(double alpha) const;
  double angle_pid(double delta);

  /**
   * @brief FSSIM风格滑移角计算
   * 计算车辆质心的滑移角 beta = atan(vy / vx)
   * @return 滑移角 [rad]
   */
  double computeSlipAngle() const;

  /**
   * @brief FSSIM风格滑移角补偿
   * 在纯追踪转向角基础上补偿滑移角影响
   * @param delta 原始转向角 [rad]
   * @return 补偿后的转向角 [rad]
   */
  double compensateSlipAngle(double delta) const;

  /**
   * @brief 计算速度自适应前视距离
   * 低速时使用较短前视距离，高速时使用较长前视距离
   * @return 前视距离 [m]
   */
  double computeAdaptiveLookahead() const;

  /**
   * @brief 使用前视点计算转向角
   * @param target_index 目标路径点索引
   * @return 转向角值（已转换为车辆转向指令）
   */
  int ComputeSteeringWithLookahead(int target_index);

  /**
   * @brief 获取指定路径点的目标速度
   * @param index 路径点索引
   * @return 目标速度 [m/s]，无数据时返回 default_target_speed_
   */
  double GetTargetSpeedAt(int index) const;

  /**
   * @brief 获取指定路径点的曲率
   * @param index 路径点索引
   * @return 曲率 [1/m]，无数据时返回 0.0
   */
  double GetCurvatureAt(int index) const;

  /**
   * @brief 默认油门计算（可被子类覆盖）
   */
  virtual int ComputeDefaultPedal();

  /**
   * @brief 默认刹车计算（可被子类覆盖）
   */
  virtual int ComputeDefaultBrake();

  /**
   * @brief 默认状态计算（可被子类覆盖）
   */
  virtual int ComputeDefaultStatus();

  void RequestStop();

  std::vector<Position> path_coordinate_{};
  std::vector<double> target_speeds_{};
  std::vector<double> curvatures_{};

  double car_x_{0.0};
  double car_y_{0.0};
  double car_theta_{0.0};
  double car_veloc_{0.0};
  double car_fangle_{0.0};

  // FSSIM风格扩展状态
  double car_vy_{0.0};        // 横向速度 [m/s]
  double car_yaw_rate_{0.0};  // 偏航角速度 [rad/s]

  double lookhead_{0.0};
  double angle_kv_{0.0};
  double angle_kl_{2.0};
  double angle_kp_{1.0};
  double angle_ki_{0.0};
  double angle_kd_{0.0};
  double steering_delta_max_{0.5};
  double car_length_{1.55};

  // FSSIM风格车辆参数
  double cg_to_front_{0.77};
  double cg_to_rear_{0.78};
  double mass_{190.0};
  bool enable_slip_compensation_{true};
  double slip_gain_{0.5};
  double min_lookahead_{2.0};
  double max_lookahead_{10.0};
  double curvature_ff_gain_{1.0};

  // Steering conversion parameters
  double steering_ratio_{3.73};
  int steering_offset_{110};

  double angle_integra_{0.0};
  double veloc_integra_{0.0};
  double last_angle_error_{0.0};
  double last_steering_output_{0.0};  // For rate limiting

  int tar_{0};
  int now_{0};

  bool finish_signal_{false};
  bool stop_requested_{false};

  // B25: default pedal tuning constants (extracted from ComputeDefaultPedal)
  double default_target_speed_{4.0};
  double default_pedal_kp_{0.5};
  double default_pedal_ki_{0.1};
  double default_pedal_cap_{5.0};
  double default_pedal_max_{30.0};
  double default_min_speed_threshold_{0.3};
  double default_high_speed_threshold_{2.0};

  // B27: active braking tuning constants
  double brake_kp_{40.0};           // 制动比例增益
  double brake_max_{80.0};          // 最大制动力
  double brake_speed_margin_{0.5};  // 超速容忍量 [m/s]
};

}  // namespace control_core

#endif  // CONTROL_CORE_CONTROLLER_BASE_HPP_
