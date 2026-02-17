#ifndef PLANNING_CORE_SKIDPAD_DETECTION_CORE_HPP_
#define PLANNING_CORE_SKIDPAD_DETECTION_CORE_HPP_

#include <cmath>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "planning_core/line_detection_core.hpp"

namespace planning_core
{

struct SkidpadParams
{
  double circle2lidar{15.0};
  double targetX{16.0};
  double targetY{-1.0};
  double FinTargetX{40.0};
  double FinTargetY{0.0};
  double distanceThreshold{0.5};
  double leavedistanceThreshold{1.0};
  int inverse_flag{1};
  double stopdistance{5.0};

  // 圆弧与车辆参数
  double circle_radius{9.125};
  double car_length{1.87};
  double path_interval{0.05};
  double center_distance_nominal{18.25};
  double center_distance_tolerance{6.0};

  // Circle fitting
  int fit_ransac_iterations{120};
  double fit_inlier_threshold{0.35};
  int fit_min_inliers{4};
  double fit_radius_min{5.0};
  double fit_radius_max{20.0};

  // Phase machine
  int phase_min_dwell_frames{5};
  double phase_entry_switch_dist{1.2};
  double phase_crossover_switch_dist{1.5};
  double phase_exit_switch_dist{2.0};

  // Phase-aware speed caps
  double speed_entry{6.0};
  double speed_warmup{7.0};
  double speed_timed{8.0};
  double speed_crossover{6.5};
  double speed_exit{5.0};

  // PassThrough 滤波器限制
  double passthrough_x_min{0.1};
  double passthrough_x_max{15.0};
  double passthrough_y_min{-3.0};
  double passthrough_y_max{3.0};
};

enum class SkidpadPhase
{
  ENTRY = 0,
  RIGHT_WARMUP = 1,
  RIGHT_TIMED = 2,
  CROSSOVER = 3,
  LEFT_WARMUP = 4,
  LEFT_TIMED = 5,
  EXIT = 6
};

struct Trajectory
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
};

struct PointComparator
{
  bool operator()(const pcl::PointXYZ &a, const pcl::PointXYZ &b) const
  {
    if (a.x < b.x)
      return true;
    if (a.x > b.x)
      return false;
    if (a.y < b.y)
      return true;
    if (a.y > b.y)
      return false;
    if (a.z < b.z)
      return true;
    return false;
  }
};

class SkidpadDetectionCore
{
public:
  explicit SkidpadDetectionCore(const SkidpadParams &params);

  void SetParams(const SkidpadParams &params);
  void ProcessConeDetections(const std::vector<ConePoint> &cones);
  void UpdateVehicleState(const Trajectory &state);

  void RunAlgorithm();

  bool HasNewPath() const { return path_updated_; }
  void ClearPathUpdated() { path_updated_ = false; }
  const std::vector<Pose> &GetPath() const { return path_output_; }
  bool IsApproachingGoal() const { return approaching_goal_; }
  SkidpadPhase GetPhase() const { return phase_; }
  double GetRecommendedSpeedCap() const;
  std::string GetPhaseName() const;
  int GetRightLaps() const { return right_laps_; }
  int GetLeftLaps() const { return left_laps_; }
  bool IsGeometryValid() const { return geometry_.valid; }

private:
  struct CircleModel
  {
    Eigen::Vector2d center{0.0, 0.0};
    double radius{0.0};
    int inliers{0};
    bool valid{false};
  };

  struct GeometryModel
  {
    CircleModel right;
    CircleModel left;
    bool valid{false};
  };

  void PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_ptr);
  bool EstimateGeometry(GeometryModel *geometry);
  bool FitCircleRansac(const std::vector<Eigen::Vector2d> &points, CircleModel *model) const;
  static bool CircleFromThreePoints(const Eigen::Vector2d &a,
                                    const Eigen::Vector2d &b,
                                    const Eigen::Vector2d &c,
                                    Eigen::Vector2d *center,
                                    double *radius);
  void UpdatePhaseMachine();
  void TransitionTo(SkidpadPhase next_phase);
  void UpdateCircleProgress(const Eigen::Vector2d &center,
                            double *prev_angle,
                            bool *angle_initialized,
                            double *accumulated_angle,
                            int *lap_count);
  void UpdateApproaching();
  std::vector<Pose> BuildPhasePath() const;
  std::vector<Pose> BuildFallbackPath() const;
  void AppendLine(const Eigen::Vector2d &a,
                  const Eigen::Vector2d &b,
                  std::vector<Pose> *path) const;
  void AppendArc(const Eigen::Vector2d &center,
                 double radius,
                 double start_angle,
                 double delta_angle,
                 std::vector<Pose> *path) const;
  static double NormalizeAngle(double angle);
  Eigen::Vector2d LocalToWorld(const Eigen::Vector2d &local) const;
  Eigen::Vector2d CurrentWorldPosition() const;

  SkidpadParams params_{};
  Trajectory current_pose_{};
  std::vector<Pose> path_output_{};
  bool path_updated_{false};
  bool approaching_goal_{false};

  pcl::PointCloud<pcl::PointXYZ>::Ptr skidpad_msg_ptr_;
  std::vector<Eigen::Vector2d> cones_local_{};
  GeometryModel geometry_{};

  SkidpadPhase phase_{SkidpadPhase::ENTRY};
  int phase_dwell_frames_{0};

  int right_laps_{0};
  int left_laps_{0};
  double right_accum_angle_{0.0};
  double left_accum_angle_{0.0};
  double prev_right_angle_{0.0};
  double prev_left_angle_{0.0};
  bool right_angle_initialized_{false};
  bool left_angle_initialized_{false};
};

} // namespace planning_core

#endif // PLANNING_CORE_SKIDPAD_DETECTION_CORE_HPP_
