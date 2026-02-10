#include <localization_core/factor_graph_optimizer.hpp>
#include <localization_core/circle_constraint_factor.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>

#include <chrono>
#include <cstdio>
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

namespace {
constexpr uint8_t kConeBlue = 0;
constexpr uint8_t kConeNone = 4;
constexpr uint8_t kConeRed = 5;

inline bool isBoundaryColor(uint8_t c) {
  return c == kConeBlue || c == kConeRed;
}
}  // namespace

// ─── Construction / Configuration ──────────────────────────────

FactorGraphOptimizer::FactorGraphOptimizer(const FactorGraphConfig& cfg)
    : cfg_(cfg),
      anomaly_sm_(cfg.anomaly),
      descriptor_reloc_(cfg.reloc),
      particle_reloc_(cfg.reloc) {
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

  // Reset anomaly & relocalization
  anomaly_sm_.Reset();
  descriptor_reloc_.ClearDatabase();
  particle_reloc_.Reset();
  matched_count_ = 0;
  total_obs_count_ = 0;
  last_chi2_normalized_ = 0.0;
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
  addGeometryPriorFactors();

  // Add initial values for new keyframe
  Symbol pose_key(fg_symbols::kPose, keyframe_idx_);
  Symbol vel_key(fg_symbols::kVelocity, keyframe_idx_);

  new_values_->insert(pose_key,
      gtsam::Pose2(current_pose.x, current_pose.y, current_pose.theta));
  new_values_->insert(vel_key, Vector2(last_speed_, 0.0));

  // Run optimization
  runOptimization();

  // Compute health metrics and evaluate anomaly state
  last_chi2_normalized_ = computeChi2Normalized();
  const double match_ratio = computeConeMatchRatio();
  anomaly_sm_.Evaluate(last_chi2_normalized_, match_ratio,
                       gnss_quality_, timestamp);

  // Build and store descriptor for relocalization database
  if (anomaly_sm_.GetState() == AnomalyState::TRACKING) {
    auto desc_vec = descriptor_reloc_.BuildDescriptor(landmarks_, opt_pose_);
    SubMapDescriptor smd;
    smd.keyframe_id = keyframe_idx_;
    smd.px = opt_pose_.x;
    smd.py = opt_pose_.y;
    smd.ptheta = opt_pose_.theta;
    smd.histogram = std::move(desc_vec);

    // Store local landmark IDs and positions for RANSAC verification
    const double r2 = cfg_.reloc.submap_radius * cfg_.reloc.submap_radius;
    for (const auto& lm : landmarks_) {
      const double dx = lm.x - opt_pose_.x;
      const double dy = lm.y - opt_pose_.y;
      if (dx * dx + dy * dy <= r2) {
        smd.local_landmark_ids.push_back(lm.id);
        smd.local_landmark_positions.emplace_back(lm.x, lm.y);
      }
    }

    descriptor_reloc_.AddToDatabase(smd);
  }

  // ─── Execute relocalization when in RELOC states ──────────────
  const AnomalyState astate = anomaly_sm_.GetState();

  if (astate == AnomalyState::RELOC_A) {
    // Descriptor-based relocalization
    auto cur_desc = descriptor_reloc_.BuildDescriptor(landmarks_, opt_pose_);

    // Collect local landmarks for RANSAC
    std::vector<FgLandmark> local_lms;
    const double r2 = cfg_.reloc.submap_radius * cfg_.reloc.submap_radius;
    for (const auto& lm : landmarks_) {
      const double dx = lm.x - opt_pose_.x;
      const double dy = lm.y - opt_pose_.y;
      if (dx * dx + dy * dy <= r2) {
        local_lms.push_back(lm);
      }
    }

    RelocResult rr = descriptor_reloc_.TryRelocalize(cur_desc, local_lms, opt_pose_);
    if (rr.success) {
      // Apply correction: shift the optimized pose
      opt_pose_.x += rr.dx;
      opt_pose_.y += rr.dy;
      opt_pose_.theta += rr.dtheta;
      opt_pose_.theta = std::atan2(std::sin(opt_pose_.theta),
                                    std::cos(opt_pose_.theta));
      fprintf(stderr, "[FG] RELOC_A success: dx=%.2f dy=%.2f dtheta=%.3f inliers=%d\n",
              rr.dx, rr.dy, rr.dtheta, rr.inlier_count);
      // Force anomaly SM back to TRACKING on next evaluation
      anomaly_sm_.ForceState(AnomalyState::TRACKING);
    }
  } else if (astate == AnomalyState::RELOC_B) {
    // Particle-filter relocalization
    if (particle_reloc_.GetParticles().empty()) {
      // Initialize particles around current (possibly drifted) pose
      particle_reloc_.Initialize(opt_pose_);
    }

    // Predict with accumulated IMU
    if (accum_dt_ > 0.0) {
      particle_reloc_.Predict(last_speed_, accum_dtheta_ / std::max(accum_dt_, 1e-6), accum_dt_);
    }

    // Update with cone observations
    if (!cone_obs_.empty()) {
      particle_reloc_.Update(cone_obs_, landmarks_);
    }

    if (particle_reloc_.HasConverged()) {
      Pose2 reloc_pose = particle_reloc_.GetEstimate();
      opt_pose_ = reloc_pose;
      fprintf(stderr, "[FG] RELOC_B converged: x=%.2f y=%.2f theta=%.3f\n",
              reloc_pose.x, reloc_pose.y, reloc_pose.theta);
      anomaly_sm_.ForceState(AnomalyState::TRACKING);
      particle_reloc_.Reset();
    }
  }

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
  // Reset per-keyframe match counters
  matched_count_ = 0;
  total_obs_count_ = 0;

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

    // Track landmark count before to distinguish match vs creation
    const size_t lm_count_before = landmarks_.size();

    // Find or create landmark using mapper's reference pose for global coords
    int lm_idx = findOrCreateLandmark(obs, ref_pose);
    total_obs_count_++;
    if (lm_idx < 0) continue;

    // Only count as "matched" if we associated with an existing landmark,
    // not if we created a new one
    if (landmarks_.size() == lm_count_before) {
      matched_count_++;
    }

    Symbol lm_key(fg_symbols::kLandmark, static_cast<uint64_t>(lm_idx));

    // Range-bearing noise (distance-dependent)
    double sr = cfg_.sigma_range_base + cfg_.sigma_range_scale * obs.range;
    double sb = cfg_.sigma_bearing_base + cfg_.sigma_bearing_scale * obs.range;

    // Color-topology soft association: inflate range sigma based on confusion
    sr += addColorTopologyFactor(obs, lm_idx);

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

  // Search existing landmarks using multi-dimensional cost
  double best_cost = cfg_.gate_threshold;
  int best_idx = -1;
  for (size_t i = 0; i < landmarks_.size(); ++i) {
    const double dx = landmarks_[i].x - gx;
    const double dy = landmarks_[i].y - gy;
    const double d2 = dx * dx + dy * dy;

    // Skip if beyond merge distance (fast reject)
    if (d2 > cfg_.merge_distance * cfg_.merge_distance) continue;

    // Multi-dimensional cost: w_maha * d² + w_color * color_cost + w_topo * topo_cost
    double cost = cfg_.w_maha * d2;

    // Color confusion cost
    const uint8_t obs_c = obs.color_type;
    const uint8_t lm_c = landmarks_[i].color_type;
    if (obs_c < kColorTypeCount && lm_c < kColorTypeCount) {
      // Use confusion matrix: higher P(obs|true) → lower cost
      cost += cfg_.w_color * (1.0 - cfg_.color_confusion[obs_c][lm_c]);
    }

    // Topology cost (HUAT convention): left=RED, right=BLUE.
    if (obs_c != kConeNone && lm_c != kConeNone && obs_c != lm_c) {
      // Different colors on same side → penalty
      if ((obs_c == kConeBlue && lm_c == kConeRed) || (obs_c == kConeRed && lm_c == kConeBlue)) {
        cost += cfg_.w_topo * cfg_.topo_penalty;
      }
    }

    if (cost < best_cost) {
      best_cost = cost;
      best_idx = static_cast<int>(i);
    }
  }

  if (best_idx >= 0) {
    landmarks_[best_idx].obs_count++;
    // Update color if previously unknown
    if (landmarks_[best_idx].color_type == kConeNone && obs.color_type != kConeNone) {
      landmarks_[best_idx].color_type = obs.color_type;
    }
    return best_idx;
  }

  // Create new landmark
  if (static_cast<int>(landmarks_.size()) >= cfg_.max_landmarks) {
    fprintf(stderr, "[FG] Landmark limit reached (%d), dropping new observation\n", cfg_.max_landmarks);
    return -1;
  }

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

  // Add circle constraint once for skidpad mode
  if (cfg_.map_mode == "skidpad") {
    gtsam::Point2 c1(cfg_.circle_center1_x, cfg_.circle_center1_y);
    gtsam::Point2 c2(cfg_.circle_center2_x, cfg_.circle_center2_y);
    auto circle_noise = Diagonal::Sigmas(
        (gtsam::Vector(1) << cfg_.circle_sigma).finished());
    new_factors_->emplace_shared<CircleConstraintFactor>(
        lm_key, c1, c2, cfg_.circle_radius, circle_noise);
  }

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

// ─── New methods ────────────────────────────────────────────────

AnomalyState FactorGraphOptimizer::GetAnomalyState() const {
  return anomaly_sm_.GetState();
}

double FactorGraphOptimizer::addColorTopologyFactor(
    const ConeObservation& obs, int lm_idx) {
  if (lm_idx < 0 || lm_idx >= static_cast<int>(landmarks_.size())) return 0.0;

  double sigma_inflate = 0.0;
  const uint8_t obs_c = obs.color_type;
  const uint8_t lm_c = landmarks_[lm_idx].color_type;

  // Color confusion cost: use confusion matrix to compute soft penalty
  if (obs_c < kColorTypeCount && lm_c < kColorTypeCount) {
    const double p_match = cfg_.color_confusion[obs_c][lm_c];
    // Lower probability → higher sigma inflation
    sigma_inflate += cfg_.color_weight * (1.0 - p_match) * cfg_.color_mismatch_penalty;
  }

  // Topology penalty (HUAT): red/blue boundary mismatch.
  if (obs_c != kConeNone && lm_c != kConeNone) {
    if ((obs_c == kConeBlue && lm_c == kConeRed) || (obs_c == kConeRed && lm_c == kConeBlue)) {
      sigma_inflate += cfg_.topo_penalty * cfg_.w_topo;
    }
  }

  return sigma_inflate;
}

void FactorGraphOptimizer::addGeometryPriorFactors() {
  // Circle constraints are now added once per landmark at creation time
  // in findOrCreateLandmark(). This method is kept for future
  // non-per-landmark geometry priors (e.g. lane-width constraints).
}

double FactorGraphOptimizer::computeChi2Normalized() {
  if (!isam_ || keyframe_idx_ == 0) return 0.0;

  // Compute proper per-DOF chi²: sum of squared whitened residuals / total DOF
  try {
    gtsam::Values result = isam_->calculateEstimate();
    const auto& factors = isam_->getFactorsUnsafe();
    double total_chi2 = 0.0;
    int total_dof = 0;
    for (size_t i = 0; i < factors.size(); ++i) {
      auto nf = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factors[i]);
      if (!nf) continue;
      // unwhitenedError → whitenedError via noise model
      gtsam::Vector uw = nf->unwhitenedError(result);
      gtsam::Vector w = nf->noiseModel()->whiten(uw);
      total_chi2 += w.squaredNorm();
      total_dof += static_cast<int>(w.size());
    }
    return (total_dof > 0) ? total_chi2 / total_dof : 0.0;
  } catch (...) {
    return 0.0;
  }
}

double FactorGraphOptimizer::computeConeMatchRatio() {
  return (total_obs_count_ > 0)
      ? static_cast<double>(matched_count_) / total_obs_count_
      : 0.0;
}

TopoRelation FactorGraphOptimizer::classifyRelation(
    const ConeObservation& obs, int lm_idx,
    const Pose2& pose) const {
  if (lm_idx < 0 || lm_idx >= static_cast<int>(landmarks_.size()))
    return TopoRelation::UNKNOWN;

  // Classify based on bearing: left side (bearing > 0) vs right side.
  // HUAT convention: left=RED, right=BLUE.
  const double bearing = obs.bearing;
  const uint8_t lm_color = landmarks_[lm_idx].color_type;

  if (lm_color == kConeNone || obs.color_type == kConeNone) return TopoRelation::UNKNOWN;

  // Same color type → SAME_SIDE
  if (obs.color_type == lm_color) return TopoRelation::SAME_SIDE;

  if (!isBoundaryColor(lm_color) || !isBoundaryColor(obs.color_type)) {
    return TopoRelation::UNKNOWN;
  }

  // If observation and landmark have different boundary colors, check side consistency.
  const bool obs_left = (bearing > 0);
  const bool lm_is_red = (lm_color == kConeRed);
  const bool lm_is_blue = (lm_color == kConeBlue);
  const bool consistent = (obs_left && lm_is_red) || (!obs_left && lm_is_blue);

  return consistent ? TopoRelation::SAME_SIDE : TopoRelation::OPPOSITE_SIDE;
}

}  // namespace localization_core
