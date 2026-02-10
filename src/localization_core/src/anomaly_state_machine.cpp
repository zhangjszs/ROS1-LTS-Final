#include <localization_core/anomaly_state_machine.hpp>

#include <algorithm>
#include <numeric>

namespace localization_core {

AnomalyStateMachine::AnomalyStateMachine(const AnomalyConfig& cfg)
    : cfg_(cfg) {}

void AnomalyStateMachine::Reset() {
  state_ = AnomalyState::TRACKING;
  chi2_window_.clear();
  low_match_start_ = -1.0;
  no_cone_frames_ = 0;
  reloc_start_time_ = -1.0;
}

void AnomalyStateMachine::ForceState(AnomalyState s) {
  state_ = s;
  if (s == AnomalyState::TRACKING) {
    low_match_start_ = -1.0;
    reloc_start_time_ = -1.0;
    chi2_window_.clear();
  }
}

double AnomalyStateMachine::windowMean() const {
  if (chi2_window_.empty()) return 0.0;
  const double sum = std::accumulate(chi2_window_.begin(), chi2_window_.end(), 0.0);
  return sum / static_cast<double>(chi2_window_.size());
}

double AnomalyStateMachine::GetCovarianceScale() const {
  switch (state_) {
    case AnomalyState::TRACKING:  return 1.0;
    case AnomalyState::DEGRADED:  return 3.0;
    case AnomalyState::LOST:      return 10.0;
    case AnomalyState::RELOC_A:   return 10.0;
    case AnomalyState::RELOC_B:   return 10.0;
    default:                      return 1.0;
  }
}

void AnomalyStateMachine::Evaluate(double chi2_normalized, double match_ratio,
                                    GnssQuality gnss_quality, double timestamp) {
  // Update chi² sliding window
  chi2_window_.push_back(chi2_normalized);
  while (static_cast<int>(chi2_window_.size()) > cfg_.chi2_window) {
    chi2_window_.pop_front();
  }

  const double chi2_avg = windowMean();

  // Track low match ratio duration
  if (match_ratio < cfg_.match_ratio_lost) {
    if (low_match_start_ < 0.0) low_match_start_ = timestamp;
  } else {
    low_match_start_ = -1.0;
  }

  const double low_match_duration =
      (low_match_start_ >= 0.0) ? (timestamp - low_match_start_) : 0.0;

  switch (state_) {
    case AnomalyState::TRACKING:
      if (chi2_avg > cfg_.chi2_degrade) {
        state_ = AnomalyState::DEGRADED;
      }
      break;

    case AnomalyState::DEGRADED:
      if (chi2_avg < cfg_.chi2_recover) {
        state_ = AnomalyState::TRACKING;
        low_match_start_ = -1.0;
      } else if (low_match_duration > cfg_.match_lost_duration) {
        state_ = AnomalyState::LOST;
      }
      break;

    case AnomalyState::LOST:
      // Transition to descriptor-based relocalization
      state_ = AnomalyState::RELOC_A;
      reloc_start_time_ = timestamp;
      break;

    case AnomalyState::RELOC_A:
      // If chi² recovers, go back to TRACKING
      if (chi2_avg < cfg_.chi2_recover && match_ratio > cfg_.match_ratio_lost) {
        state_ = AnomalyState::TRACKING;
        low_match_start_ = -1.0;
        reloc_start_time_ = -1.0;
      }
      // If descriptor reloc times out, escalate to particle filter
      else if (reloc_start_time_ > 0.0 &&
               (timestamp - reloc_start_time_) > cfg_.reloc_a_timeout) {
        state_ = AnomalyState::RELOC_B;
        reloc_start_time_ = timestamp;
      }
      break;

    case AnomalyState::RELOC_B:
      // If chi² recovers, go back to TRACKING
      if (chi2_avg < cfg_.chi2_recover && match_ratio > cfg_.match_ratio_lost) {
        state_ = AnomalyState::TRACKING;
        low_match_start_ = -1.0;
        reloc_start_time_ = -1.0;
      }
      // If particle reloc times out, go back to LOST to retry
      else if (reloc_start_time_ > 0.0 &&
               (timestamp - reloc_start_time_) > cfg_.reloc_b_timeout) {
        state_ = AnomalyState::LOST;
        reloc_start_time_ = -1.0;
      }
      break;
  }
}

}  // namespace localization_core
