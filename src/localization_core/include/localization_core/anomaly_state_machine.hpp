#pragma once

#include <localization_core/factor_graph_types.hpp>

#include <deque>

namespace localization_core {

/// Five-state anomaly detector for factor graph health monitoring.
///
/// States: TRACKING → DEGRADED → LOST → RELOC_A → RELOC_B → TRACKING
///
/// Transitions are driven by chi² residual, cone match ratio, and GNSS quality.
class AnomalyStateMachine {
 public:
  explicit AnomalyStateMachine(const AnomalyConfig& cfg = {});

  /// Evaluate sensor health and advance state machine.
  /// @param chi2_normalized  normalized chi² from iSAM2
  /// @param match_ratio      fraction of observed cones matched to landmarks
  /// @param gnss_quality     current GNSS quality tier
  /// @param timestamp        monotonic seconds
  void Evaluate(double chi2_normalized, double match_ratio,
                GnssQuality gnss_quality, double timestamp);

  /// Current anomaly state.
  AnomalyState GetState() const { return state_; }

  /// Covariance scale factor (1.0 = normal, >1 = inflated).
  double GetCovarianceScale() const;

  /// Reset to TRACKING.
  void Reset();

  /// Force a specific state (used by relocalization on success).
  void ForceState(AnomalyState s);

 private:
  AnomalyConfig cfg_;
  AnomalyState state_ = AnomalyState::TRACKING;

  // Chi² sliding window
  std::deque<double> chi2_window_;

  // Low match ratio timer
  double low_match_start_ = -1.0;

  // No-cone frame counter
  int no_cone_frames_ = 0;

  // Relocalization timer
  double reloc_start_time_ = -1.0;

  double windowMean() const;
};

}  // namespace localization_core
