#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

#include "localization_core/descriptor_relocator.hpp"
#include "localization_core/factor_graph_types.hpp"
#include "localization_core/particle_relocator.hpp"
#include "localization_core/types.hpp"

using namespace localization_core;

namespace {

std::vector<FgLandmark> generateLandmarks(int n, double spread, uint32_t seed = 42) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-spread, spread);
  std::vector<FgLandmark> lms;
  lms.reserve(n);
  for (int i = 0; i < n; ++i) {
    FgLandmark lm;
    lm.id = static_cast<uint32_t>(i);
    lm.x = dist(rng);
    lm.y = dist(rng);
    lm.color_type = static_cast<uint8_t>(i % 3);
    lm.obs_count = 5;
    lms.push_back(lm);
  }
  return lms;
}

std::vector<ConeObservation> generateObservations(int n, uint32_t seed = 123) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> range_dist(1.0, 20.0);
  std::uniform_real_distribution<double> bearing_dist(-M_PI, M_PI);
  std::vector<ConeObservation> obs;
  obs.reserve(n);
  for (int i = 0; i < n; ++i) {
    ConeObservation co;
    co.range = range_dist(rng);
    co.bearing = bearing_dist(rng);
    co.color_type = static_cast<uint8_t>(i % 3);
    co.confidence = 0.8;
    obs.push_back(co);
  }
  return obs;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════
// Data Association Performance (brute-force nearest-neighbor)
// ═══════════════════════════════════════════════════════════════

TEST(LocalizationPerf, DataAssociation_20obs_200lm) {
  const int N_OBS = 20;
  const int N_LM = 200;
  auto landmarks = generateLandmarks(N_LM, 50.0);
  auto observations = generateObservations(N_OBS);
  Pose2 pose{0, 0, 0};

  auto start = std::chrono::high_resolution_clock::now();
  const int ITERS = 100;
  for (int iter = 0; iter < ITERS; ++iter) {
    for (const auto& obs : observations) {
      double gx = pose.x + obs.range * std::cos(obs.bearing);
      double gy = pose.y + obs.range * std::sin(obs.bearing);
      double best_d2 = 1e18;
      int best_id = -1;
      for (const auto& lm : landmarks) {
        double dx = lm.x - gx;
        double dy = lm.y - gy;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
          best_d2 = d2;
          best_id = static_cast<int>(lm.id);
        }
      }
      (void)best_id;
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(end - start).count() / ITERS;
  EXPECT_LT(ms, 2.0) << "Data association took " << ms << " ms (limit: 2.0 ms)";
}

// ═══════════════════════════════════════════════════════════════
// Descriptor Build Performance
// ═══════════════════════════════════════════════════════════════

TEST(LocalizationPerf, DescriptorBuild_200lm) {
  RelocConfig cfg;
  cfg.submap_radius = 30.0;
  cfg.n_sectors = 12;
  cfg.n_rings = 5;
  cfg.n_channels = 3;
  DescriptorRelocator reloc(cfg);

  auto landmarks = generateLandmarks(200, 25.0);
  Pose2 pose{0, 0, 0};

  auto start = std::chrono::high_resolution_clock::now();
  const int ITERS = 1000;
  for (int i = 0; i < ITERS; ++i) {
    auto desc = reloc.BuildDescriptor(landmarks, pose);
    (void)desc;
  }
  auto end = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(end - start).count() / ITERS;
  EXPECT_LT(ms, 1.0) << "Descriptor build took " << ms << " ms (limit: 1.0 ms)";
}

// ═══════════════════════════════════════════════════════════════
// Descriptor Match Performance
// ═══════════════════════════════════════════════════════════════

TEST(LocalizationPerf, DescriptorMatch_100db) {
  RelocConfig cfg;
  cfg.submap_radius = 30.0;
  cfg.n_sectors = 12;
  cfg.n_rings = 5;
  cfg.n_channels = 3;
  cfg.max_descriptors = 200;
  cfg.top_k = 5;
  cfg.ransac_max_iter = 50;
  cfg.min_inliers = 3;
  cfg.ransac_inlier_thresh = 1.5;
  DescriptorRelocator reloc(cfg);

  // Populate database
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> pos_dist(-50, 50);
  for (int i = 0; i < 100; ++i) {
    auto lms = generateLandmarks(30, 15.0, 100 + i);
    Pose2 p{pos_dist(rng), pos_dist(rng), 0.0};
    auto desc = reloc.BuildDescriptor(lms, p);
    SubMapDescriptor sd;
    sd.keyframe_id = i;
    sd.px = p.x;
    sd.py = p.y;
    sd.ptheta = p.theta;
    sd.histogram = desc;
    for (const auto& lm : lms) {
      sd.local_landmark_ids.push_back(lm.id);
      sd.local_landmark_positions.emplace_back(lm.x, lm.y);
    }
    reloc.AddToDatabase(sd);
  }

  auto query_lms = generateLandmarks(20, 15.0, 999);
  Pose2 query_pose{5.0, 5.0, 0.1};
  auto query_desc = reloc.BuildDescriptor(query_lms, query_pose);

  auto start = std::chrono::high_resolution_clock::now();
  const int ITERS = 50;
  for (int i = 0; i < ITERS; ++i) {
    auto result = reloc.TryRelocalize(query_desc, query_lms, query_pose);
    (void)result;
  }
  auto end = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(end - start).count() / ITERS;
  EXPECT_LT(ms, 10.0) << "Descriptor match took " << ms << " ms (limit: 10.0 ms)";
}

// ═══════════════════════════════════════════════════════════════
// Particle Filter Update Performance
// ═══════════════════════════════════════════════════════════════

TEST(LocalizationPerf, ParticleFilterUpdate_200p_10obs_100lm) {
  RelocConfig cfg;
  cfg.n_particles = 200;
  cfg.particle_sigma_xy = 3.0;
  cfg.particle_sigma_theta = 0.3;
  ParticleRelocator reloc(cfg);
  reloc.Initialize({0, 0, 0});

  auto landmarks = generateLandmarks(100, 30.0);
  auto observations = generateObservations(10);

  auto start = std::chrono::high_resolution_clock::now();
  const int ITERS = 100;
  for (int i = 0; i < ITERS; ++i) {
    reloc.Update(observations, landmarks);
  }
  auto end = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(end - start).count() / ITERS;
  EXPECT_LT(ms, 5.0) << "Particle filter update took " << ms << " ms (limit: 5.0 ms)";
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
