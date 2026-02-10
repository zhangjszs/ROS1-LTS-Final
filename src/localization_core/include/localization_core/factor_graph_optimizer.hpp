#pragma once

#include <localization_core/factor_graph_types.hpp>
#include <localization_core/anomaly_state_machine.hpp>
#include <localization_core/descriptor_relocator.hpp>
#include <localization_core/particle_relocator.hpp>
#include <localization_core/types.hpp>

#include <cstdint>
#include <memory>
#include <vector>

// Forward-declare GTSAM types to avoid header pollution
namespace gtsam {
class ISAM2;
class NonlinearFactorGraph;
class Values;
}  // namespace gtsam

namespace localization_core {

/// iSAM2-based factor graph optimizer for FSD localization.
///
/// Encapsulates:
///   FG-3  IMU preintegration (BetweenFactor<Pose2> + velocity)
///   FG-4  GNSS weak prior
///   FG-5  Vehicle speed / yaw-rate factors
///   FG-6  iSAM2 incremental optimization + landmark management
///
/// Thread-safety: NOT thread-safe. Caller must synchronize.
class FactorGraphOptimizer {
 public:
  explicit FactorGraphOptimizer(const FactorGraphConfig& cfg);
  ~FactorGraphOptimizer();

  // Non-copyable
  FactorGraphOptimizer(const FactorGraphOptimizer&) = delete;
  FactorGraphOptimizer& operator=(const FactorGraphOptimizer&) = delete;

  /// Reconfigure (does NOT reset graph state).
  void Configure(const FactorGraphConfig& cfg);

  /// Reset all state (graph, landmarks, keyframe counter).
  void Reset();

  // ─── Sensor input ────────────────────────────────────────────

  /// Accumulate a velocity/yaw-rate measurement for preintegration.
  /// Call this at IMU rate (e.g. 100 Hz).
  /// @param v_forward  forward speed [m/s] (from INS)
  /// @param wz  yaw rate [rad/s]
  /// @param dt  time since last sample [s]
  void AddImuMeasurement(double v_forward, double wz, double dt);

  /// Provide a GNSS position observation (ENU).
  void SetGnssObservation(double x, double y, GnssQuality quality);

  /// Provide vehicle speed observation.
  void SetSpeedObservation(double v_forward);

  /// Provide cone observations for the current step.
  void SetConeObservations(const std::vector<ConeObservation>& obs);

  // ─── Update cycle ────────────────────────────────────────────

  /// Try to create a new keyframe and run iSAM2 update.
  /// @param current_pose  current dead-reckoned pose (for keyframe check)
  /// @param timestamp     seconds since start
  /// @return true if a keyframe was created and graph was optimized
  bool TryUpdate(const Pose2& current_pose, double timestamp);

  // ─── Output ──────────────────────────────────────────────────

  /// Latest optimized pose.
  Pose2 GetOptimizedPose() const;

  /// Latest optimized velocity (vx, vy).
  void GetOptimizedVelocity(double& vx, double& vy) const;

  /// All landmarks in the map.
  std::vector<FgLandmark> GetLandmarks() const;

  /// Number of keyframes in the graph.
  uint64_t NumKeyframes() const;

  /// Whether the optimizer has been initialized.
  bool IsInitialized() const;

  /// Last optimization wall-clock time in milliseconds.
  double LastOptTimeMs() const;

  /// Current anomaly state.
  AnomalyState GetAnomalyState() const;

  /// Last computed normalized chi² value.
  double GetChi2Normalized() const { return last_chi2_normalized_; }

  /// Last computed cone match ratio.
  double GetConeMatchRatio() const {
    return (total_obs_count_ > 0)
        ? static_cast<double>(matched_count_) / total_obs_count_
        : 0.0;
  }

 private:
  // ─── Internal helpers ────────────────────────────────────────

  void initializeFirstKeyframe(const Pose2& pose, double timestamp);
  bool shouldCreateKeyframe(const Pose2& pose, double timestamp) const;
  void addImuFactor();
  void addGnssFactor();
  void addSpeedFactor();
  void addConeFactors(const Pose2& ref_pose);
  int findOrCreateLandmark(const ConeObservation& obs, const Pose2& pose);
  void runOptimization();

  // New methods for color-topology association and anomaly detection
  double addColorTopologyFactor(const ConeObservation& obs, int lm_idx);
  void addGeometryPriorFactors();
  double computeChi2Normalized();
  double computeConeMatchRatio();
  TopoRelation classifyRelation(const ConeObservation& obs, int lm_idx,
                                 const Pose2& pose) const;

  // ─── State ───────────────────────────────────────────────────

  FactorGraphConfig cfg_;

  // GTSAM objects (pimpl to keep header clean)
  std::unique_ptr<gtsam::ISAM2> isam_;
  std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors_;
  std::unique_ptr<gtsam::Values> new_values_;

  // Keyframe bookkeeping
  uint64_t keyframe_idx_ = 0;
  bool initialized_ = false;
  Pose2 last_keyframe_pose_;
  double last_keyframe_time_ = 0.0;

  // Velocity/yaw-rate preintegration accumulator
  double accum_dx_ = 0.0;
  double accum_dy_ = 0.0;
  double accum_dtheta_ = 0.0;
  double accum_dt_ = 0.0;
  double last_speed_ = 0.0;  // latest speed for velocity factor

  // Current-step sensor data (consumed on TryUpdate)
  bool has_gnss_ = false;
  double gnss_x_ = 0.0;
  double gnss_y_ = 0.0;
  GnssQuality gnss_quality_ = GnssQuality::INVALID;

  bool has_speed_ = false;
  double speed_obs_ = 0.0;

  std::vector<ConeObservation> cone_obs_;

  // Landmark database
  std::vector<FgLandmark> landmarks_;
  uint32_t next_landmark_id_ = 0;

  // Latest optimized state
  Pose2 opt_pose_;
  double opt_vx_ = 0.0;
  double opt_vy_ = 0.0;

  // Diagnostics
  double last_opt_time_ms_ = 0.0;

  // Anomaly detection & relocalization
  AnomalyStateMachine anomaly_sm_;
  DescriptorRelocator descriptor_reloc_;
  ParticleRelocator particle_reloc_;
  int matched_count_ = 0;
  int total_obs_count_ = 0;
  double last_chi2_normalized_ = 0.0;
};

}  // namespace localization_core
