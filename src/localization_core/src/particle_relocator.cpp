#include <localization_core/particle_relocator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace localization_core {

ParticleRelocator::ParticleRelocator(const RelocConfig& cfg)
    : cfg_(cfg) {}

void ParticleRelocator::Reset() {
  particles_.clear();
  initialized_ = false;
}

void ParticleRelocator::Initialize(const Pose2& seed_pose) {
  particles_.resize(cfg_.n_particles);
  std::mt19937 rng(std::random_device{}());
  std::normal_distribution<double> dist_xy(0.0, cfg_.particle_sigma_xy);
  std::normal_distribution<double> dist_theta(0.0, cfg_.particle_sigma_theta);

  const double uniform_weight = 1.0 / cfg_.n_particles;
  for (auto& p : particles_) {
    p.x = seed_pose.x + dist_xy(rng);
    p.y = seed_pose.y + dist_xy(rng);
    p.theta = seed_pose.theta + dist_theta(rng);
    p.weight = uniform_weight;
    p.log_weight = std::log(uniform_weight);
  }
  initialized_ = true;
}

void ParticleRelocator::Predict(double v_forward, double wz, double dt) {
  if (!initialized_ || dt <= 0.0) return;

  std::mt19937 rng(std::random_device{}());
  std::normal_distribution<double> noise_xy(0.0, 0.05 * std::abs(v_forward) * dt + 0.01);
  std::normal_distribution<double> noise_theta(0.0, 0.02 * std::abs(wz) * dt + 0.005);

  for (auto& p : particles_) {
    const double mid_theta = p.theta + wz * dt * 0.5;
    p.x += std::cos(mid_theta) * v_forward * dt + noise_xy(rng);
    p.y += std::sin(mid_theta) * v_forward * dt + noise_xy(rng);
    p.theta += wz * dt + noise_theta(rng);
  }
}

void ParticleRelocator::Update(
    const std::vector<ConeObservation>& observations,
    const std::vector<FgLandmark>& landmarks) {
  if (!initialized_ || observations.empty() || landmarks.empty()) return;

  for (auto& p : particles_) {
    double log_likelihood = 0.0;
    const double cp = std::cos(p.theta);
    const double sp = std::sin(p.theta);

    for (const auto& obs : observations) {
      if (obs.range <= 0.0) continue;

      // Expected global position of observed cone from this particle
      const double lx = obs.range * std::cos(obs.bearing);
      const double ly = obs.range * std::sin(obs.bearing);
      const double gx = p.x + cp * lx - sp * ly;
      const double gy = p.y + sp * lx + cp * ly;

      // Find nearest landmark
      double min_d2 = std::numeric_limits<double>::max();
      for (const auto& lm : landmarks) {
        const double dx = lm.x - gx;
        const double dy = lm.y - gy;
        min_d2 = std::min(min_d2, dx * dx + dy * dy);
      }

      // Gaussian likelihood: exp(-d²/(2σ²))
      const double sigma = 2.0;
      log_likelihood += -min_d2 / (2.0 * sigma * sigma);
    }

    p.log_weight += log_likelihood;
  }

  normalize();
  resample();
}

void ParticleRelocator::normalize() {
  // Log-sum-exp normalization
  double max_log = -std::numeric_limits<double>::max();
  for (const auto& p : particles_) {
    max_log = std::max(max_log, p.log_weight);
  }

  double sum_exp = 0.0;
  for (auto& p : particles_) {
    p.weight = std::exp(p.log_weight - max_log);
    sum_exp += p.weight;
  }

  if (sum_exp > 0.0) {
    for (auto& p : particles_) {
      p.weight /= sum_exp;
      p.log_weight = std::log(p.weight);
    }
  }
}

void ParticleRelocator::resample() {
  // Systematic resampling
  const int n = static_cast<int>(particles_.size());
  if (n == 0) return;

  std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> u01(0.0, 1.0 / n);
  double r = u01(rng);

  std::vector<Particle> new_particles(n);
  double cumulative = particles_[0].weight;
  int j = 0;

  for (int i = 0; i < n; ++i) {
    const double target = r + static_cast<double>(i) / n;
    while (cumulative < target && j < n - 1) {
      ++j;
      cumulative += particles_[j].weight;
    }
    new_particles[i] = particles_[j];
    new_particles[i].weight = 1.0 / n;
    new_particles[i].log_weight = std::log(1.0 / n);
  }

  // Add diffusion noise
  std::normal_distribution<double> diff_xy(0.0, 0.1);
  std::normal_distribution<double> diff_theta(0.0, 0.02);
  for (auto& p : new_particles) {
    p.x += diff_xy(rng);
    p.y += diff_xy(rng);
    p.theta += diff_theta(rng);
  }

  particles_ = std::move(new_particles);
}

bool ParticleRelocator::HasConverged() const {
  if (!initialized_ || particles_.empty()) return false;

  // Compute weighted mean
  double mx = 0, my = 0, ms = 0, mc = 0;
  for (const auto& p : particles_) {
    mx += p.weight * p.x;
    my += p.weight * p.y;
    ms += p.weight * std::sin(p.theta);
    mc += p.weight * std::cos(p.theta);
  }

  // Compute weighted standard deviation
  double var_x = 0, var_y = 0, var_theta = 0;
  const double mean_theta = std::atan2(ms, mc);
  for (const auto& p : particles_) {
    var_x += p.weight * (p.x - mx) * (p.x - mx);
    var_y += p.weight * (p.y - my) * (p.y - my);
    double dt = std::atan2(std::sin(p.theta - mean_theta),
                           std::cos(p.theta - mean_theta));
    var_theta += p.weight * dt * dt;
  }

  const double std_xy = std::sqrt(var_x + var_y);
  const double std_theta = std::sqrt(var_theta);

  return std_xy < cfg_.converge_sigma_xy &&
         std_theta < cfg_.converge_sigma_theta;
}

Pose2 ParticleRelocator::GetEstimate() const {
  Pose2 est;
  if (!initialized_ || particles_.empty()) return est;

  double mx = 0, my = 0, ms = 0, mc = 0;
  for (const auto& p : particles_) {
    mx += p.weight * p.x;
    my += p.weight * p.y;
    ms += p.weight * std::sin(p.theta);
    mc += p.weight * std::cos(p.theta);
  }

  est.x = mx;
  est.y = my;
  est.theta = std::atan2(ms, mc);
  return est;
}

}  // namespace localization_core
