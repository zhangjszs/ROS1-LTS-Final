#include <localization_core/factor_graph_optimizer.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>

#include <chrono>
#include <algorithm>
#include <cmath>

namespace localization_core {

using gtsam::Symbol;
using gtsam::Point2;
using gtsam::Vector2;
using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Robust;
using gtsam::noiseModel::mEstimator::Huber;
using gtsam::noiseModel::mEstimator::Cauchy;

// ─── Construction / Configuration ──────────────────────────────

FactorGraphOptimizer::FactorGraphOptimizer(const FactorGraphConfig& cfg)
    : cfg_(cfg) {
  Reset();
}

FactorGraphOptimizer::~FactorGraphOptimizer() = default;

void FactorGraphOptimizer::Configure(const FactorGraphConfig& cfg) {
  cfg_ = cfg;
}

void FactorGraphOptimizer::Reset() {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = cfg_.relinearize_threshold;
  params.relinearizeSkip = cfg_.relinearize_skip;
  if (cfg_.use_dogleg) {
    params.optimizationParams = gtsam::ISAM2DoglegParams();
  }

  isam_ = std::make_unique<gtsam::ISAM2>(params);
  new_factors_ = std::make_unique<gtsam::NonlinearFactorGraph>();
  new_values_ = std::make_unique<gtsam::Values>();

  keyframe_idx_ = 0;
  initialized_ = false;
  last_keyframe_pose_ = {};
  last_keyframe_time_ = 0.0;

  accum_dx_ = accum_dy_ = accum_dtheta_ = 0.0;
  accum_dt_ = 0.0;
  last_speed_ = 0.0;

  has_gnss_ = false;
  has_speed_ = false;
  cone_obs_.clear();

  landmarks_.clear();
  next_landmark_id_ = 0;

  opt_pose_ = {};
  opt_vx_ = opt_vy_ = 0.0;
  last_opt_time_ms_ = 0.0;
}

// ─── Sensor input ──────────────────────────────────────────────

void FactorGraphOptimizer::AddImuMeasurement(double v_forward,
                                              double wz, double dt) {
  if (dt <= 0.0) return;

  // Velocity-yaw-rate dead reckoning in body frame
  // Integrate position in world-relative delta using mid-point heading
  const double mid_theta = accum_dtheta_ + wz * dt * 0.5;
  const double c = std::cos(mid_theta);
  const double s = std::sin(mid_theta);

  accum_dx_ += c * v_forward * dt;
  accum_dy_ += s * v_forward * dt;
  accum_dtheta_ += wz * dt;
  accum_dt_ += dt;
  last_speed_ = v_forward;
}

void FactorGraphOptimizer::SetGnssObservation(double x, double y,
                                               GnssQuality quality) {
  has_gnss_ = true;
  gnss_x_ = x;
  gnss_y_ = y;
  gnss_quality_ = quality;
}

void FactorGraphOptimizer::SetSpeedObservation(double v_forward) {
  has_speed_ = true;
  speed_obs_ = v_forward;
}

void FactorGraphOptimizer::SetConeObservations(
    const std::vector<ConeObservation>& obs) {
  cone_obs_ = obs;
}

// ─── Update cycle ──────────────────────────────────────────────

bool FactorGraphOptimizer::TryUpdate(const localization_core::Pose2& current_pose,
                                      double timestamp) {
  if (!initialized_) {
    initializeFirstKeyframe(current_pose, timestamp);
    return true;
  }

  if (!shouldCreateKeyframe(current_pose, timestamp)) {
    return false;
  }

  // Create new keyframe
  keyframe_idx_++;

  // Add factors
  addImuFactor();
  addGnssFactor();
  addSpeedFactor();
  addConeFactors(current_pose);

  // Add initial values for new keyframe
  Symbol pose_key(fg_symbols::kPose, keyframe_idx_);
  Symbol vel_key(fg_symbols::kVelocity, keyframe_idx_);

  new_values_->insert(pose_key,
      gtsam::Pose2(current_pose.x, current_pose.y, current_pose.theta));
  new_values_->insert(vel_key, Vector2(last_speed_, 0.0));

  // Run optimization
  runOptimization();

  // Update bookkeeping
  last_keyframe_pose_ = opt_pose_;
  last_keyframe_time_ = timestamp;

  // Reset accumulators
  accum_dx_ = accum_dy_ = accum_dtheta_ = 0.0;
  accum_dt_ = 0.0;
  has_gnss_ = false;
  has_speed_ = false;
  cone_obs_.clear();

  return true;
}

// ─── Output ────────────────────────────────────────────────────

localization_core::Pose2 FactorGraphOptimizer::GetOptimizedPose() const {
  return opt_pose_;
}

void FactorGraphOptimizer::GetOptimizedVelocity(double& vx, double& vy) const {
  vx = opt_vx_;
  vy = opt_vy_;
}

std::vector<FgLandmark> FactorGraphOptimizer::GetLandmarks() const {
  return landmarks_;
}

uint64_t FactorGraphOptimizer::NumKeyframes() const {
  return keyframe_idx_ + (initialized_ ? 1 : 0);
}

bool FactorGraphOptimizer::IsInitialized() const {
  return initialized_;
}

double FactorGraphOptimizer::LastOptTimeMs() const {
  return last_opt_time_ms_;
}

// ─── Internal helpers ──────────────────────────────────────────

void FactorGraphOptimizer::initializeFirstKeyframe(
    const localization_core::Pose2& pose, double timestamp) {
  Symbol pose_key(fg_symbols::kPose, 0);
  Symbol vel_key(fg_symbols::kVelocity, 0);

  gtsam::Pose2 gtsam_pose(pose.x, pose.y, pose.theta);

  // Strong prior on first pose
  auto pose_noise = Diagonal::Sigmas(
      (gtsam::Vector(3) << 0.1, 0.1, 0.05).finished());
  new_factors_->addPrior(pose_key, gtsam_pose, pose_noise);

  // Velocity prior (assume stationary or slow)
  auto vel_noise = Diagonal::Sigmas(Vector2(1.0, 1.0));
  new_factors_->addPrior(vel_key, Vector2(0.0, 0.0), vel_noise);

  new_values_->insert(pose_key, gtsam_pose);
  new_values_->insert(vel_key, Vector2(0.0, 0.0));

  runOptimization();

  last_keyframe_pose_ = pose;
  last_keyframe_time_ = timestamp;
  opt_pose_ = pose;
  initialized_ = true;

  // Reset accumulators
  accum_dx_ = accum_dy_ = accum_dtheta_ = 0.0;
  accum_dt_ = 0.0;
  has_gnss_ = false;
  has_speed_ = false;
  cone_obs_.clear();
}

bool FactorGraphOptimizer::shouldCreateKeyframe(
    const localization_core::Pose2& pose, double timestamp) const {
  const double dx = pose.x - last_keyframe_pose_.x;
  const double dy = pose.y - last_keyframe_pose_.y;
  const double dist = std::sqrt(dx * dx + dy * dy);
  if (dist >= cfg_.keyframe.dist_threshold) return true;

  const double dyaw = std::abs(
      std::atan2(std::sin(pose.theta - last_keyframe_pose_.theta),
                 std::cos(pose.theta - last_keyframe_pose_.theta)));
  if (dyaw >= cfg_.keyframe.yaw_threshold) return true;

  if ((timestamp - last_keyframe_time_) >= cfg_.keyframe.dt_threshold) return true;

  return false;
}

// FG-3: IMU preintegration factor
void FactorGraphOptimizer::addImuFactor() {
  if (accum_dt_ <= 0.0) return;

  Symbol prev_pose(fg_symbols::kPose, keyframe_idx_ - 1);
  Symbol curr_pose(fg_symbols::kPose, keyframe_idx_);

  // BetweenFactor<Pose2> approximating IMU preintegration
  gtsam::Pose2 delta(accum_dx_, accum_dy_, accum_dtheta_);
  auto noise = Diagonal::Sigmas(
      (gtsam::Vector(3) << cfg_.sigma_imu_xy * std::sqrt(accum_dt_),
                           cfg_.sigma_imu_xy * std::sqrt(accum_dt_),
                           cfg_.sigma_imu_theta * std::sqrt(accum_dt_))
          .finished());
  new_factors_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
      prev_pose, curr_pose, delta, noise);

  // Velocity continuity factor (assume roughly constant speed between keyframes)
  Symbol prev_vel(fg_symbols::kVelocity, keyframe_idx_ - 1);
  Symbol curr_vel(fg_symbols::kVelocity, keyframe_idx_);
  Vector2 delta_vel(0.0, 0.0);
  auto vel_noise = Diagonal::Sigmas(
      Vector2(cfg_.sigma_vel * std::sqrt(accum_dt_),
              cfg_.sigma_vel * std::sqrt(accum_dt_)));
  new_factors_->emplace_shared<gtsam::BetweenFactor<Vector2>>(
      prev_vel, curr_vel, delta_vel, vel_noise);
}

// FG-4: GNSS weak prior factor
void FactorGraphOptimizer::addGnssFactor() {
  if (!has_gnss_ || gnss_quality_ == GnssQuality::INVALID) return;

  double sigma;
  switch (gnss_quality_) {
    case GnssQuality::GOOD:   sigma = cfg_.sigma_gnss_good; break;
    case GnssQuality::MEDIUM: sigma = cfg_.sigma_gnss_medium; break;
    case GnssQuality::POOR:   sigma = cfg_.sigma_gnss_poor; break;
    default: return;
  }

  Symbol pose_key(fg_symbols::kPose, keyframe_idx_);

  // Prior on full Pose2 with very loose heading (only constraining xy)
  gtsam::Pose2 gnss_pose(gnss_x_, gnss_y_, opt_pose_.theta);
  auto base_noise = Diagonal::Sigmas(
      (gtsam::Vector(3) << sigma, sigma, 100.0).finished());  // 100 rad = ~unconstrained heading
  auto robust_noise = Robust::Create(Cauchy::Create(cfg_.cauchy_gnss), base_noise);
  new_factors_->addPrior(pose_key, gnss_pose, robust_noise);
}

// FG-5: Vehicle speed / yaw-rate factors
void FactorGraphOptimizer::addSpeedFactor() {
  if (!has_speed_) return;

  Symbol vel_key(fg_symbols::kVelocity, keyframe_idx_);

  // Speed prior: v_forward ≈ vx (assuming small slip angle)
  Vector2 speed_vec(speed_obs_, 0.0);
  auto base_noise = Diagonal::Sigmas(
      Vector2(cfg_.sigma_speed, cfg_.sigma_speed * 2.0));
  auto robust_noise = Robust::Create(Huber::Create(cfg_.huber_speed), base_noise);
  new_factors_->addPrior(vel_key, speed_vec, robust_noise);
}

// FG-5 + cone observation factors
void FactorGraphOptimizer::addConeFactors(const localization_core::Pose2& ref_pose) {
  if (cone_obs_.empty()) return;

  Symbol pose_key(fg_symbols::kPose, keyframe_idx_);

  std::vector<const ConeObservation*> valid_obs;
  valid_obs.reserve(cone_obs_.size());
  for (const auto& obs : cone_obs_) {
    if (obs.range <= 0.0 || obs.range > cfg_.new_landmark_range_max) continue;
    if (obs.confidence < cfg_.min_cone_confidence) continue;
    valid_obs.push_back(&obs);
  }
  if (valid_obs.empty()) return;

  std::sort(valid_obs.begin(), valid_obs.end(),
            [](const ConeObservation* a, const ConeObservation* b) {
              if (a->range == b->range) {
                return a->confidence > b->confidence;
              }
              return a->range < b->range;
            });

  if (cfg_.max_cone_factors_per_keyframe > 0 &&
      valid_obs.size() > static_cast<size_t>(cfg_.max_cone_factors_per_keyframe)) {
    valid_obs.resize(static_cast<size_t>(cfg_.max_cone_factors_per_keyframe));
  }

  for (const auto* obs_ptr : valid_obs) {
    const auto& obs = *obs_ptr;

    // Find or create landmark using mapper's reference pose for global coords
    int lm_idx = findOrCreateLandmark(obs, ref_pose);
    if (lm_idx < 0) continue;

    Symbol lm_key(fg_symbols::kLandmark, static_cast<uint64_t>(lm_idx));

    // Range-bearing noise (distance-dependent)
    double sr = cfg_.sigma_range_base + cfg_.sigma_range_scale * obs.range;
    double sb = cfg_.sigma_bearing_base + cfg_.sigma_bearing_scale * obs.range;

    // Color mismatch penalty added to range noise
    if (obs.color_type != 4 && landmarks_[lm_idx].color_type != 4 &&
        obs.color_type != landmarks_[lm_idx].color_type) {
      sr += cfg_.color_mismatch_penalty * cfg_.color_weight;
    }

    auto base_noise = Diagonal::Sigmas(Vector2(sb, sr));
    auto robust_noise = Robust::Create(Huber::Create(cfg_.huber_cone), base_noise);

    new_factors_->emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, Point2>>(
        pose_key, lm_key,
        gtsam::Rot2(obs.bearing), obs.range,
        robust_noise);
  }
}

int FactorGraphOptimizer::findOrCreateLandmark(const ConeObservation& obs,
                                                const localization_core::Pose2& pose) {
  // Convert observation to global coordinates for matching
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  const double lx_body = obs.range * std::cos(obs.bearing);
  const double ly_body = obs.range * std::sin(obs.bearing);
  const double gx = pose.x + c * lx_body - s * ly_body;
  const double gy = pose.y + s * lx_body + c * ly_body;

  // Search existing landmarks
  double best_dist_sq = cfg_.merge_distance * cfg_.merge_distance;
  int best_idx = -1;
  for (size_t i = 0; i < landmarks_.size(); ++i) {
    const double dx = landmarks_[i].x - gx;
    const double dy = landmarks_[i].y - gy;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_dist_sq) {
      best_dist_sq = d2;
      best_idx = static_cast<int>(i);
    }
  }

  if (best_idx >= 0) {
    landmarks_[best_idx].obs_count++;
    // Update color if previously unknown
    if (landmarks_[best_idx].color_type == 4 && obs.color_type != 4) {
      landmarks_[best_idx].color_type = obs.color_type;
    }
    return best_idx;
  }

  // Create new landmark
  if (static_cast<int>(landmarks_.size()) >= cfg_.max_landmarks) return -1;

  FgLandmark lm;
  lm.id = next_landmark_id_++;
  lm.x = gx;
  lm.y = gy;
  lm.color_type = obs.color_type;
  lm.obs_count = 1;
  landmarks_.push_back(lm);

  int idx = static_cast<int>(landmarks_.size()) - 1;
  Symbol lm_key(fg_symbols::kLandmark, static_cast<uint64_t>(idx));

  // Add initial value
  new_values_->insert(lm_key, Point2(gx, gy));

  // Add weak prior on new landmark
  auto lm_noise = Diagonal::Sigmas(
      Vector2(cfg_.landmark_init_sigma, cfg_.landmark_init_sigma));
  new_factors_->addPrior(lm_key, Point2(gx, gy), lm_noise);

  return idx;
}

void FactorGraphOptimizer::runOptimization() {
  auto t0 = std::chrono::steady_clock::now();

  isam_->update(*new_factors_, *new_values_);
  if (cfg_.run_extra_update) {
    isam_->update();
  }

  // Extract results
  gtsam::Values result = isam_->calculateEstimate();

  Symbol pose_key(fg_symbols::kPose, keyframe_idx_);
  Symbol vel_key(fg_symbols::kVelocity, keyframe_idx_);

  if (result.exists(pose_key)) {
    auto p = result.at<gtsam::Pose2>(pose_key);
    opt_pose_.x = p.x();
    opt_pose_.y = p.y();
    opt_pose_.theta = p.theta();
  }
  if (result.exists(vel_key)) {
    auto v = result.at<Vector2>(vel_key);
    opt_vx_ = v(0);
    opt_vy_ = v(1);
  }

  // Update landmark positions from optimization
  for (size_t i = 0; i < landmarks_.size(); ++i) {
    Symbol lm_key(fg_symbols::kLandmark, static_cast<uint64_t>(i));
    if (result.exists(lm_key)) {
      auto pt = result.at<Point2>(lm_key);
      landmarks_[i].x = pt.x();
      landmarks_[i].y = pt.y();
    }
  }

  // Clear pending factors/values
  new_factors_->resize(0);
  new_values_->clear();

  auto t1 = std::chrono::steady_clock::now();
  last_opt_time_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

}  // namespace localization_core
