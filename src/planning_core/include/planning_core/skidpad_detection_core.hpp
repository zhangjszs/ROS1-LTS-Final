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

private:
  void PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_ptr);
  bool ChangPathFlag(double current_x, double current_y, double TargetX_, double TargetY_, double DistanceThreshold_);
  void ChangLeavePathFlag(double current_x, double current_y, double TargetX_, double TargetY_, double LeaveDistanceThreshold_);
  void UpdateApproaching(double current_x, double current_y, double FinTargetX_, double FInTargetY_, double stopdistance_);
  std::vector<Pose> TransformPath(const std::vector<Pose> &path) const;

  SkidpadParams params_{};
  Trajectory current_pose_{};
  std::vector<Pose> path_output_{};
  bool path_updated_{false};
  bool approaching_goal_{false};

  pcl::PointCloud<pcl::PointXYZ>::Ptr skidpad_msg_ptr_;
  std::set<pcl::PointXYZ, PointComparator> orderCloud_{};
  std::set<pcl::PointXYZ, PointComparator> orderCloud_real_{};
  pcl::PointXYZ pc_trans_{};

  int inverse_flag_{1};
  int modeFlag_{0};
  double circle2lidar_{15.0};
  double at2_angle_mid_{0.0};
  double prev_at2_angle_mid_{0.0};
  double targetX_{16.0};
  double targetY_{-1.0};
  double FinTargetX_{40.0};
  double FInTargetY_{0.0};
  double distanceThreshold_{0.5};
  double LeavedistanceThreshold_{1.0};
  double stopdistance_{5.0};
  bool at2_angle_calced_{false};
  bool find_four_bucket_{false};
  bool matchFlag_{true};
  bool changFlag_{false};
  bool leaveFlag_{false};
  bool haschanged_{true};
  bool skipIteration_{false};
  double mid_x_fir_{0.0};
  double mid_y_fir_{0.0};
  double mid_x_sec_{0.0};
  double mid_y_sec_{0.0};

public:
  double lipu{0.0};
};

} // namespace planning_core

#endif // PLANNING_CORE_SKIDPAD_DETECTION_CORE_HPP_
