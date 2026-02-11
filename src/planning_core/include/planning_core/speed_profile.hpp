#ifndef PLANNING_CORE_SPEED_PROFILE_HPP_
#define PLANNING_CORE_SPEED_PROFILE_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

namespace planning_core
{

struct SpeedProfileParams
{
  double speed_cap = 20.0;
  double max_lateral_acc = 6.5;
  double max_accel = 3.0;
  double max_brake = 4.0;
  double min_speed = 1.0;
  double curvature_epsilon = 1e-3;
  double current_speed = 0.0;
};

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

inline double PointDist(const Point2D &a, const Point2D &b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double SignedCurvature3(const Point2D &p0, const Point2D &p1, const Point2D &p2)
{
  const double a = PointDist(p0, p1);
  const double b = PointDist(p1, p2);
  const double c = PointDist(p0, p2);
  const double denom = a * b * c;
  if (denom < 1e-6)
    return 0.0;
  const double cross = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
  return 2.0 * cross / denom;
}
/// Compute signed curvatures for a path of 2D points.
/// Output vector has same size as input; endpoints copy from neighbors.
inline void ComputeCurvatures(const std::vector<Point2D> &path,
                              std::vector<double> &curvatures)
{
  const size_t n = path.size();
  curvatures.assign(n, 0.0);
  if (n < 3)
    return;
  for (size_t i = 1; i + 1 < n; ++i)
  {
    curvatures[i] = SignedCurvature3(path[i - 1], path[i], path[i + 1]);
  }
  curvatures.front() = curvatures[1];
  curvatures.back() = curvatures[n - 2];
}

/// Compute a feasible speed profile given curvatures and path geometry.
/// Uses lateral-acceleration limit, forward (accel) and backward (brake) passes.
inline void ComputeSpeedProfile(const std::vector<Point2D> &path,
                                const std::vector<double> &curvatures,
                                const SpeedProfileParams &p,
                                std::vector<double> &target_speeds)
{
  const size_t n = path.size();
  target_speeds.assign(n, 0.0);
  if (n == 0)
    return;

  const double v_cap = std::max(0.0, p.speed_cap);
  const double a_lat = std::max(0.1, p.max_lateral_acc);
  const double a_acc = std::max(0.0, p.max_accel);
  const double a_brake = std::max(0.0, p.max_brake);
  const double kappa_eps = std::max(1e-6, p.curvature_epsilon);
  const double v_min = std::max(0.0, std::min(p.min_speed, v_cap));

  // Segment lengths
  std::vector<double> ds;
  if (n >= 2)
  {
    ds.resize(n - 1, 1e-3);
    for (size_t i = 0; i + 1 < n; ++i)
      ds[i] = std::max(1e-3, PointDist(path[i], path[i + 1]));
  }

  // Lateral acceleration constraint
  std::vector<double> v_ref(n, v_cap);
  for (size_t i = 0; i < n; ++i)
  {
    const double denom = std::max(std::abs(curvatures[i]), kappa_eps);
    const double v_lat_max = std::sqrt(std::max(0.0, a_lat / denom));
    v_ref[i] = std::max(v_min, std::min(v_cap, v_lat_max));
  }

  // Seed with current vehicle speed
  v_ref[0] = std::max(v_min, std::min(v_cap,
      std::isfinite(p.current_speed) ? std::max(0.0, p.current_speed) : 0.0));

  // Forward pass (acceleration constraint)
  for (size_t i = 1; i < n; ++i)
  {
    const double reachable = std::sqrt(std::max(0.0,
        v_ref[i - 1] * v_ref[i - 1] + 2.0 * a_acc * ds[i - 1]));
    v_ref[i] = std::min(v_ref[i], reachable);
  }

  // Backward pass (braking constraint)
  for (size_t i = n - 1; i > 0; --i)
  {
    const double reachable = std::sqrt(std::max(0.0,
        v_ref[i] * v_ref[i] + 2.0 * a_brake * ds[i - 1]));
    v_ref[i - 1] = std::min(v_ref[i - 1], reachable);
  }

  // Sanitize output
  for (size_t i = 0; i < n; ++i)
  {
    target_speeds[i] = (std::isfinite(v_ref[i]) && v_ref[i] > 0.0) ? v_ref[i] : 0.0;
  }
}

// APPEND_MARKER

} // namespace planning_core

#endif // PLANNING_CORE_SPEED_PROFILE_HPP_
