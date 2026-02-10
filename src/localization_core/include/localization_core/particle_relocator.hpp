#pragma once

#include <localization_core/factor_graph_types.hpp>
#include <localization_core/types.hpp>

#include <vector>

namespace localization_core {

/// Particle-filter based relocalization (RELOC_B channel).
///
/// Used when descriptor-based relocalization fails. Spreads particles
/// around the last known pose and converges using cone observations.
class ParticleRelocator {
 public:
  explicit ParticleRelocator(const RelocConfig& cfg = {});

  /// Initialize particles around a seed pose with Gaussian spread.
  void Initialize(const Pose2& seed_pose);

  /// Predict step: propagate particles using IMU dead-reckoning.
  void Predict(double v_forward, double wz, double dt);

  /// Update step: weight particles by cone observation likelihood.
  /// @param observations  cone observations in body frame (range, bearing)
  /// @param landmarks     global landmark map for matching
  void Update(const std::vector<ConeObservation>& observations,
              const std::vector<FgLandmark>& landmarks);

  /// Check if particles have converged.
  bool HasConverged() const;

  /// Get the weighted mean pose estimate.
  Pose2 GetEstimate() const;

  /// Get all particles (for debugging).
  const std::vector<Particle>& GetParticles() const { return particles_; }

  /// Reset the particle filter.
  void Reset();

 private:
  RelocConfig cfg_;
  std::vector<Particle> particles_;
  bool initialized_ = false;

  void resample();
  void normalize();
};

}  // namespace localization_core
