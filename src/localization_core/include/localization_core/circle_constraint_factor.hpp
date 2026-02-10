#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <algorithm>
#include <cmath>

namespace localization_core {

/// Unary factor constraining a Point2 landmark to lie on one of two circles.
///
/// Error = min(|dist(p, c1) - R|, |dist(p, c2) - R|)
///
/// Used for Skidpad mode where cones must lie on one of the two figure-8 circles.
class CircleConstraintFactor : public gtsam::NoiseModelFactor1<gtsam::Point2> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Point2>;

  CircleConstraintFactor(gtsam::Key key,
                         const gtsam::Point2& center1,
                         const gtsam::Point2& center2,
                         double radius,
                         const gtsam::SharedNoiseModel& model)
      : Base(model, key),
        center1_(center1),
        center2_(center2),
        radius_(radius) {}

  ~CircleConstraintFactor() override = default;

  gtsam::Vector evaluateError(
      const gtsam::Point2& p,
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    const double dx1 = p.x() - center1_.x();
    const double dy1 = p.y() - center1_.y();
    const double dist1 = std::sqrt(dx1 * dx1 + dy1 * dy1);

    const double dx2 = p.x() - center2_.x();
    const double dy2 = p.y() - center2_.y();
    const double dist2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

    const double err1 = std::abs(dist1 - radius_);
    const double err2 = std::abs(dist2 - radius_);

    // Choose the circle that gives smaller error
    const bool use_c1 = (err1 <= err2);
    const double dx = use_c1 ? dx1 : dx2;
    const double dy = use_c1 ? dy1 : dy2;
    const double dist = use_c1 ? dist1 : dist2;

    // Signed error: dist - radius (positive = outside circle)
    const double signed_err = dist - radius_;

    if (H) {
      // Jacobian of (dist - R) w.r.t. p = [dx/dist, dy/dist]
      const double eps = 1e-9;
      if (dist > eps) {
        *H = (gtsam::Matrix(1, 2) << dx / dist, dy / dist).finished();
      } else {
        *H = gtsam::Matrix::Zero(1, 2);
      }
    }

    return (gtsam::Vector(1) << signed_err).finished();
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new CircleConstraintFactor(*this));
  }

 private:
  gtsam::Point2 center1_;
  gtsam::Point2 center2_;
  double radius_;
};

}  // namespace localization_core
