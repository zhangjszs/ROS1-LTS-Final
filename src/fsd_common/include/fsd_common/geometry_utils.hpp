#pragma once
#include <cmath>

namespace fsd_common {

inline constexpr double DegToRad(double deg) { return deg * M_PI / 180.0; }
inline constexpr double RadToDeg(double rad) { return rad * 180.0 / M_PI; }

inline double Distance2D(double x1, double y1, double x2, double y2) {
  const double dx = x2 - x1, dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

inline double NormalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double AngleDiff(double a, double b) {
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

}  // namespace fsd_common
