#include "planning_core/skidpad_detection_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace planning_core
{
namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

Pose MakePose(double x, double y)
{
  Pose pose;
  pose.x = x;
  pose.y = y;
  pose.z = 0.0;
  pose.qx = 0.0;
  pose.qy = 0.0;
  pose.qz = 0.0;
  pose.qw = 1.0;
  return pose;
}
} // namespace

SkidpadDetectionCore::SkidpadDetectionCore(const SkidpadParams &params)
    : skidpad_msg_ptr_(new pcl::PointCloud<pcl::PointXYZ>())
{
  SetParams(params);
}

void SkidpadDetectionCore::SetParams(const SkidpadParams &params)
{
  params_ = params;
}

std::string SkidpadDetectionCore::GetPhaseName() const
{
  switch (phase_)
  {
  case SkidpadPhase::ENTRY:
    return "ENTRY";
  case SkidpadPhase::RIGHT_WARMUP:
    return "RIGHT_WARMUP";
  case SkidpadPhase::RIGHT_TIMED:
    return "RIGHT_TIMED";
  case SkidpadPhase::CROSSOVER:
    return "CROSSOVER";
  case SkidpadPhase::LEFT_WARMUP:
    return "LEFT_WARMUP";
  case SkidpadPhase::LEFT_TIMED:
    return "LEFT_TIMED";
  case SkidpadPhase::EXIT:
    return "EXIT";
  default:
    return "UNKNOWN";
  }
}

double SkidpadDetectionCore::GetRecommendedSpeedCap() const
{
  switch (phase_)
  {
  case SkidpadPhase::ENTRY:
    return std::max(0.0, params_.speed_entry);
  case SkidpadPhase::RIGHT_WARMUP:
  case SkidpadPhase::LEFT_WARMUP:
    return std::max(0.0, params_.speed_warmup);
  case SkidpadPhase::RIGHT_TIMED:
  case SkidpadPhase::LEFT_TIMED:
    return std::max(0.0, params_.speed_timed);
  case SkidpadPhase::CROSSOVER:
    return std::max(0.0, params_.speed_crossover);
  case SkidpadPhase::EXIT:
    return std::max(0.0, params_.speed_exit);
  default:
    return std::max(0.0, params_.speed_entry);
  }
}

void SkidpadDetectionCore::ProcessConeDetections(const std::vector<ConePoint> &cones)
{
  skidpad_msg_ptr_->clear();
  cones_local_.clear();
  cones_local_.reserve(cones.size());

  for (const auto &point : cones)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    {
      continue;
    }

    pcl::PointXYZ pcl_point;
    pcl_point.x = static_cast<float>(point.x);
    pcl_point.y = static_cast<float>(point.y);
    pcl_point.z = static_cast<float>(point.z);
    skidpad_msg_ptr_->push_back(pcl_point);
  }

  PassThrough(skidpad_msg_ptr_);
  cones_local_.reserve(skidpad_msg_ptr_->size());
  for (const auto &p : skidpad_msg_ptr_->points)
  {
    cones_local_.emplace_back(p.x, p.y);
  }
}

void SkidpadDetectionCore::UpdateVehicleState(const Trajectory &state)
{
  current_pose_ = state;
}

void SkidpadDetectionCore::PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_ptr)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(in_ptr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(params_.passthrough_x_min, params_.passthrough_x_max);
  pass.filter(*in_ptr);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(params_.passthrough_y_min, params_.passthrough_y_max);
  pass.filter(*in_ptr);
}

bool SkidpadDetectionCore::CircleFromThreePoints(const Eigen::Vector2d &a,
                                                 const Eigen::Vector2d &b,
                                                 const Eigen::Vector2d &c,
                                                 Eigen::Vector2d *center,
                                                 double *radius)
{
  const double d = 2.0 * (a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y()));
  if (std::abs(d) < 1e-8)
  {
    return false;
  }

  const double a2 = a.squaredNorm();
  const double b2 = b.squaredNorm();
  const double c2 = c.squaredNorm();

  const double ux = (a2 * (b.y() - c.y()) + b2 * (c.y() - a.y()) + c2 * (a.y() - b.y())) / d;
  const double uy = (a2 * (c.x() - b.x()) + b2 * (a.x() - c.x()) + c2 * (b.x() - a.x())) / d;

  center->x() = ux;
  center->y() = uy;
  *radius = (*center - a).norm();
  return std::isfinite(*radius) && *radius > 1e-6;
}

bool SkidpadDetectionCore::FitCircleRansac(const std::vector<Eigen::Vector2d> &points, CircleModel *model) const
{
  if (!model || points.size() < 3)
  {
    return false;
  }

  const int iterations = std::max(20, params_.fit_ransac_iterations);
  const double inlier_thr = std::max(0.05, params_.fit_inlier_threshold);
  const int min_inliers = std::max(3, params_.fit_min_inliers);
  const double r_min = std::max(0.5, params_.fit_radius_min);
  const double r_max = std::max(r_min + 0.1, params_.fit_radius_max);

  std::mt19937 rng(static_cast<uint32_t>(points.size() * 97 + 17));
  std::uniform_int_distribution<int> dist(0, static_cast<int>(points.size()) - 1);

  CircleModel best;
  std::vector<int> best_inliers;
  double best_score = std::numeric_limits<double>::infinity();

  for (int it = 0; it < iterations; ++it)
  {
    int i0 = dist(rng);
    int i1 = dist(rng);
    int i2 = dist(rng);
    if (i0 == i1 || i0 == i2 || i1 == i2)
    {
      continue;
    }

    Eigen::Vector2d center;
    double radius = 0.0;
    if (!CircleFromThreePoints(points[i0], points[i1], points[i2], &center, &radius))
    {
      continue;
    }
    if (radius < r_min || radius > r_max)
    {
      continue;
    }

    std::vector<int> inliers;
    inliers.reserve(points.size());
    double residual_sum = 0.0;
    for (size_t i = 0; i < points.size(); ++i)
    {
      const double err = std::abs((points[i] - center).norm() - radius);
      if (err <= inlier_thr)
      {
        inliers.push_back(static_cast<int>(i));
        residual_sum += err;
      }
    }

    if (static_cast<int>(inliers.size()) < min_inliers)
    {
      continue;
    }

    const double score = residual_sum / static_cast<double>(std::max<size_t>(1, inliers.size()));
    const bool better = inliers.size() > best_inliers.size() ||
                        (inliers.size() == best_inliers.size() && score < best_score);
    if (better)
    {
      best_inliers = inliers;
      best_score = score;
      best.center = center;
      best.radius = radius;
      best.inliers = static_cast<int>(inliers.size());
      best.valid = true;
    }
  }

  if (!best.valid || best.inliers < min_inliers)
  {
    return false;
  }

  // Least-squares refinement on inliers.
  Eigen::MatrixXd A(best.inliers, 3);
  Eigen::VectorXd b(best.inliers);
  for (int r = 0; r < best.inliers; ++r)
  {
    const Eigen::Vector2d &p = points[best_inliers[static_cast<size_t>(r)]];
    A(r, 0) = p.x();
    A(r, 1) = p.y();
    A(r, 2) = 1.0;
    b(r) = p.squaredNorm();
  }

  const Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
  const Eigen::Vector2d refined_center(sol(0) * 0.5, sol(1) * 0.5);
  const double refined_r2 = sol(2) + refined_center.squaredNorm();
  const double refined_radius = (refined_r2 > 0.0) ? std::sqrt(refined_r2) : best.radius;

  if (std::isfinite(refined_radius) && refined_radius >= r_min && refined_radius <= r_max)
  {
    best.center = refined_center;
    best.radius = refined_radius;
  }

  *model = best;
  return true;
}

Eigen::Vector2d SkidpadDetectionCore::LocalToWorld(const Eigen::Vector2d &local) const
{
  const double c = std::cos(current_pose_.yaw);
  const double s = std::sin(current_pose_.yaw);
  const double wx = c * local.x() - s * local.y() + current_pose_.x;
  const double wy = s * local.x() + c * local.y() + current_pose_.y;
  return Eigen::Vector2d(wx, wy);
}

Eigen::Vector2d SkidpadDetectionCore::CurrentWorldPosition() const
{
  return Eigen::Vector2d(current_pose_.x, current_pose_.y);
}

bool SkidpadDetectionCore::EstimateGeometry(GeometryModel *geometry)
{
  if (!geometry)
  {
    return false;
  }

  std::vector<Eigen::Vector2d> right_pts;
  std::vector<Eigen::Vector2d> left_pts;
  right_pts.reserve(cones_local_.size());
  left_pts.reserve(cones_local_.size());

  for (const Eigen::Vector2d &p : cones_local_)
  {
    if (p.y() < 0.0)
    {
      right_pts.push_back(p);
    }
    else
    {
      left_pts.push_back(p);
    }
  }

  CircleModel right_local;
  CircleModel left_local;
  const bool right_ok = FitCircleRansac(right_pts, &right_local);
  const bool left_ok = FitCircleRansac(left_pts, &left_local);

  if (!right_ok && !left_ok)
  {
    return false;
  }

  const double nominal_radius = std::max(0.5, params_.circle_radius);
  const double nominal_center_dist = std::max(1.0, params_.center_distance_nominal);

  if (!right_ok)
  {
    right_local = left_local;
    right_local.center.y() -= nominal_center_dist;
    right_local.radius = nominal_radius;
    right_local.valid = true;
  }

  if (!left_ok)
  {
    left_local = right_local;
    left_local.center.y() += nominal_center_dist;
    left_local.radius = nominal_radius;
    left_local.valid = true;
  }

  if (right_local.center.y() > left_local.center.y())
  {
    std::swap(right_local, left_local);
  }

  Eigen::Vector2d axis = left_local.center - right_local.center;
  double center_dist = axis.norm();
  if (!std::isfinite(center_dist) || center_dist < 1e-3)
  {
    axis = Eigen::Vector2d(0.0, 1.0);
    center_dist = 1.0;
  }
  else
  {
    axis /= center_dist;
  }

  if (std::abs(center_dist - nominal_center_dist) > std::max(0.5, params_.center_distance_tolerance))
  {
    const Eigen::Vector2d mid = 0.5 * (left_local.center + right_local.center);
    left_local.center = mid + axis * (0.5 * nominal_center_dist);
    right_local.center = mid - axis * (0.5 * nominal_center_dist);
  }

  GeometryModel est;
  est.right = right_local;
  est.left = left_local;
  est.right.radius = std::clamp(est.right.radius, params_.fit_radius_min, params_.fit_radius_max);
  est.left.radius = std::clamp(est.left.radius, params_.fit_radius_min, params_.fit_radius_max);
  if (!std::isfinite(est.right.radius))
  {
    est.right.radius = nominal_radius;
  }
  if (!std::isfinite(est.left.radius))
  {
    est.left.radius = nominal_radius;
  }

  est.right.center = LocalToWorld(est.right.center);
  est.left.center = LocalToWorld(est.left.center);
  est.valid = true;

  *geometry = est;
  return true;
}

void SkidpadDetectionCore::TransitionTo(SkidpadPhase next_phase)
{
  if (phase_ == next_phase)
  {
    return;
  }

  phase_ = next_phase;
  phase_dwell_frames_ = 0;

  if (next_phase == SkidpadPhase::RIGHT_WARMUP)
  {
    right_laps_ = 0;
    right_accum_angle_ = 0.0;
    right_angle_initialized_ = false;
  }
  else if (next_phase == SkidpadPhase::LEFT_WARMUP)
  {
    left_laps_ = 0;
    left_accum_angle_ = 0.0;
    left_angle_initialized_ = false;
  }
}

double SkidpadDetectionCore::NormalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= kTwoPi;
  }
  while (angle < -M_PI)
  {
    angle += kTwoPi;
  }
  return angle;
}

void SkidpadDetectionCore::UpdateCircleProgress(const Eigen::Vector2d &center,
                                                double *prev_angle,
                                                bool *angle_initialized,
                                                double *accumulated_angle,
                                                int *lap_count)
{
  if (!prev_angle || !angle_initialized || !accumulated_angle || !lap_count)
  {
    return;
  }

  const Eigen::Vector2d p = CurrentWorldPosition();
  const double angle = std::atan2(p.y() - center.y(), p.x() - center.x());
  if (!*angle_initialized)
  {
    *prev_angle = angle;
    *angle_initialized = true;
    return;
  }

  const double delta = NormalizeAngle(angle - *prev_angle);
  *accumulated_angle += std::abs(delta);
  *prev_angle = angle;

  while (*accumulated_angle >= (kTwoPi - 0.05))
  {
    ++(*lap_count);
    *accumulated_angle -= kTwoPi;
  }
}

void SkidpadDetectionCore::UpdatePhaseMachine()
{
  if (!geometry_.valid)
  {
    return;
  }

  ++phase_dwell_frames_;
  const int min_dwell = std::max(1, params_.phase_min_dwell_frames);

  const Eigen::Vector2d p = CurrentWorldPosition();
  const Eigen::Vector2d right_start = geometry_.right.center + Eigen::Vector2d(0.0, geometry_.right.radius);
  const Eigen::Vector2d left_start = geometry_.left.center + Eigen::Vector2d(0.0, -geometry_.left.radius);
  const double entry_dist_thr = std::max(0.2, params_.phase_entry_switch_dist);
  const double crossover_dist_thr = std::max(0.3, params_.phase_crossover_switch_dist);
  const double right_ring_residual = std::abs((p - geometry_.right.center).norm() - geometry_.right.radius);
  const double left_ring_residual = std::abs((p - geometry_.left.center).norm() - geometry_.left.radius);
  const double ring_relax_entry = std::max(0.6, entry_dist_thr * 1.8);
  const double ring_relax_crossover = std::max(0.6, crossover_dist_thr * 1.8);

  switch (phase_)
  {
  case SkidpadPhase::ENTRY:
    if (((p - right_start).norm() <= entry_dist_thr || right_ring_residual <= ring_relax_entry) &&
        phase_dwell_frames_ >= min_dwell)
    {
      TransitionTo(SkidpadPhase::RIGHT_WARMUP);
    }
    break;
  case SkidpadPhase::RIGHT_WARMUP:
    UpdateCircleProgress(geometry_.right.center,
                         &prev_right_angle_,
                         &right_angle_initialized_,
                         &right_accum_angle_,
                         &right_laps_);
    if (right_laps_ >= 1 && phase_dwell_frames_ >= min_dwell)
    {
      TransitionTo(SkidpadPhase::RIGHT_TIMED);
    }
    break;
  case SkidpadPhase::RIGHT_TIMED:
    UpdateCircleProgress(geometry_.right.center,
                         &prev_right_angle_,
                         &right_angle_initialized_,
                         &right_accum_angle_,
                         &right_laps_);
    if (right_laps_ >= 2 && phase_dwell_frames_ >= min_dwell)
    {
      TransitionTo(SkidpadPhase::CROSSOVER);
    }
    break;
  case SkidpadPhase::CROSSOVER:
    if (((p - left_start).norm() <= crossover_dist_thr || left_ring_residual <= ring_relax_crossover) &&
        phase_dwell_frames_ >= min_dwell)
    {
      TransitionTo(SkidpadPhase::LEFT_WARMUP);
    }
    break;
  case SkidpadPhase::LEFT_WARMUP:
    UpdateCircleProgress(geometry_.left.center,
                         &prev_left_angle_,
                         &left_angle_initialized_,
                         &left_accum_angle_,
                         &left_laps_);
    if (left_laps_ >= 1 && phase_dwell_frames_ >= min_dwell)
    {
      TransitionTo(SkidpadPhase::LEFT_TIMED);
    }
    break;
  case SkidpadPhase::LEFT_TIMED:
    UpdateCircleProgress(geometry_.left.center,
                         &prev_left_angle_,
                         &left_angle_initialized_,
                         &left_accum_angle_,
                         &left_laps_);
    if (left_laps_ >= 2 && phase_dwell_frames_ >= min_dwell)
    {
      TransitionTo(SkidpadPhase::EXIT);
    }
    break;
  case SkidpadPhase::EXIT:
  default:
    break;
  }
}

void SkidpadDetectionCore::AppendLine(const Eigen::Vector2d &a,
                                      const Eigen::Vector2d &b,
                                      std::vector<Pose> *path) const
{
  if (!path)
  {
    return;
  }

  const double step = std::max(0.05, params_.path_interval);
  const double dist = (b - a).norm();
  const int n = std::max(2, static_cast<int>(std::ceil(dist / step)));
  for (int i = 0; i <= n; ++i)
  {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    const Eigen::Vector2d p = a + t * (b - a);
    path->push_back(MakePose(p.x(), p.y()));
  }
}

void SkidpadDetectionCore::AppendArc(const Eigen::Vector2d &center,
                                     double radius,
                                     double start_angle,
                                     double delta_angle,
                                     std::vector<Pose> *path) const
{
  if (!path)
  {
    return;
  }

  radius = std::max(0.5, radius);
  const double step = std::max(0.05, params_.path_interval);
  const double arc_len = std::abs(delta_angle) * radius;
  const int n = std::max(30, static_cast<int>(std::ceil(arc_len / step)));

  for (int i = 0; i <= n; ++i)
  {
    const double r = static_cast<double>(i) / static_cast<double>(n);
    const double angle = start_angle + r * delta_angle;
    const double x = center.x() + radius * std::cos(angle);
    const double y = center.y() + radius * std::sin(angle);
    path->push_back(MakePose(x, y));
  }
}

std::vector<Pose> SkidpadDetectionCore::BuildFallbackPath() const
{
  std::vector<Pose> path;
  const Eigen::Vector2d start = CurrentWorldPosition();
  const Eigen::Vector2d goal(params_.targetX, params_.targetY);
  AppendLine(start, goal, &path);
  return path;
}

std::vector<Pose> SkidpadDetectionCore::BuildPhasePath() const
{
  if (!geometry_.valid)
  {
    return BuildFallbackPath();
  }

  std::vector<Pose> path;
  const Eigen::Vector2d cur = CurrentWorldPosition();

  const Eigen::Vector2d right_center = geometry_.right.center;
  const Eigen::Vector2d left_center = geometry_.left.center;
  const double right_radius = geometry_.right.radius;
  const double left_radius = geometry_.left.radius;

  const Eigen::Vector2d right_start = right_center + Eigen::Vector2d(0.0, right_radius);
  const Eigen::Vector2d left_start = left_center + Eigen::Vector2d(0.0, -left_radius);

  switch (phase_)
  {
  case SkidpadPhase::ENTRY:
    AppendLine(cur, right_start, &path);
    break;
  case SkidpadPhase::RIGHT_WARMUP:
  case SkidpadPhase::RIGHT_TIMED:
  {
    const double start_angle = std::atan2(cur.y() - right_center.y(), cur.x() - right_center.x());
    AppendArc(right_center, right_radius, start_angle, -kTwoPi, &path);
    break;
  }
  case SkidpadPhase::CROSSOVER:
    AppendLine(cur, left_start, &path);
    break;
  case SkidpadPhase::LEFT_WARMUP:
  case SkidpadPhase::LEFT_TIMED:
  {
    const double start_angle = std::atan2(cur.y() - left_center.y(), cur.x() - left_center.x());
    AppendArc(left_center, left_radius, start_angle, kTwoPi, &path);
    break;
  }
  case SkidpadPhase::EXIT:
  default:
  {
    const Eigen::Vector2d goal(params_.FinTargetX, params_.FinTargetY);
    AppendLine(cur, goal, &path);
    break;
  }
  }

  return path;
}

void SkidpadDetectionCore::UpdateApproaching()
{
  const Eigen::Vector2d p = CurrentWorldPosition();
  const Eigen::Vector2d goal(params_.FinTargetX, params_.FinTargetY);
  approaching_goal_ = ((p - goal).norm() <= std::max(0.5, params_.stopdistance));
}

void SkidpadDetectionCore::RunAlgorithm()
{
  path_updated_ = false;

  GeometryModel estimated;
  if (EstimateGeometry(&estimated))
  {
    if (geometry_.valid)
    {
      const double w = 0.25;
      estimated.right.center = (1.0 - w) * geometry_.right.center + w * estimated.right.center;
      estimated.left.center = (1.0 - w) * geometry_.left.center + w * estimated.left.center;
      estimated.right.radius = (1.0 - w) * geometry_.right.radius + w * estimated.right.radius;
      estimated.left.radius = (1.0 - w) * geometry_.left.radius + w * estimated.left.radius;
    }
    geometry_ = estimated;
  }

  if (!geometry_.valid)
  {
    path_output_ = BuildFallbackPath();
    path_updated_ = !path_output_.empty();
    UpdateApproaching();
    return;
  }

  UpdatePhaseMachine();
  path_output_ = BuildPhasePath();
  path_updated_ = !path_output_.empty();
  UpdateApproaching();
}

} // namespace planning_core
