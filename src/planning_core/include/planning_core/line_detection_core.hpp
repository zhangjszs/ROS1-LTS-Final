#ifndef PLANNING_CORE_LINE_DETECTION_CORE_HPP_
#define PLANNING_CORE_LINE_DETECTION_CORE_HPP_

#include <cmath>
#include <utility>
#include <vector>

namespace planning_core
{

struct HoughLine
{
  double rho{0.0};
  double theta{0.0};
  int votes{0};
};

struct ConePoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct VehicleState
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double v{0.0};
};

struct Pose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct LineDetectionParams
{
  double hough_rho_resolution{0.1};
  double hough_theta_resolution{0.01};
  int hough_min_votes{3};
  double theta_tolerance{0.2};
  double max_cone_distance{50.0};
  double min_cone_distance{2.0};
  double path_interval{0.1};
  double max_path_distance{75.0};
  double imu_offset_x{1.88};
  double finish_line_threshold{2.0};
  double max_cone_lateral_distance{10.0};
  double min_rho_diff{2.0};
  double max_rho_diff{20.0};
};

class LineDetectionCore
{
public:
  explicit LineDetectionCore(const LineDetectionParams &params);

  void SetParams(const LineDetectionParams &params);
  void UpdateCones(const std::vector<ConePoint> &cones);
  void UpdateVehicleState(const VehicleState &state);

  void RunAlgorithm();

  bool IsFinished() const { return finished_; }
  bool HasPlannedPath() const { return !planned_path_.empty(); }
  const std::vector<Pose> &GetPlannedPath() const { return planned_path_; }

private:
  std::vector<ConePoint> FilterCones(const std::vector<ConePoint> &cones) const;
  std::vector<HoughLine> HoughTransform(const std::vector<ConePoint> &cones) const;
  std::pair<HoughLine, HoughLine> SelectBoundaryLines(const std::vector<HoughLine> &lines) const;
  HoughLine CalculateCenterLine(const HoughLine &left_line, const HoughLine &right_line) const;
  std::vector<Pose> GeneratePath(const HoughLine &center_line) const;
  std::vector<Pose> ConvertToWorldCoordinates(const std::vector<Pose> &path) const;
  bool CheckFinishLine(double current_x, double finish_x) const;
  void InitializeAccumulator(int num_rho, int num_theta);

  LineDetectionParams params_;
  std::vector<ConePoint> cone_positions_;
  bool first_detection_done_{false};
  double finish_line_x_{0.0};
  HoughLine center_line_{};
  bool finished_{false};
  VehicleState vehicle_state_{};
  std::vector<Pose> planned_path_{};

  std::vector<std::vector<int>> hough_accumulator_;
  int cached_num_rho_{0};
  int cached_num_theta_{0};
};

} // namespace planning_core

#endif // PLANNING_CORE_LINE_DETECTION_CORE_HPP_
