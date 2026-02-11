#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "localization_core/anomaly_state_machine.hpp"
#include "localization_core/circle_constraint_factor.hpp"
#include "localization_core/descriptor_relocator.hpp"
#include "localization_core/imu_state_estimator.hpp"
#include "localization_core/particle_relocator.hpp"
#include "localization_core/location_mapper.hpp"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

using namespace localization_core;

// ═══════════════════════════════════════════════════════════════
// Existing test
// ═══════════════════════════════════════════════════════════════

TEST(LocalizationCoreTest, Initialization) {
  ImuStateEstimatorParams params;
  ImuStateEstimator estimator(params);
  EXPECT_FALSE(estimator.initialized());
}

// ═══════════════════════════════════════════════════════════════
// AnomalyStateMachine tests
// ═══════════════════════════════════════════════════════════════

class AnomalyStateMachineTest : public ::testing::Test {
 protected:
  AnomalyConfig cfg_;
  void SetUp() override {
    cfg_.chi2_degrade = 15.0;
    cfg_.chi2_recover = 5.0;
    cfg_.chi2_window = 5;
    cfg_.match_ratio_lost = 0.3;
    cfg_.match_lost_duration = 2.0;
    cfg_.reloc_a_timeout = 3.0;
    cfg_.reloc_b_timeout = 5.0;
  }
};

TEST_F(AnomalyStateMachineTest, StartsInTracking) {
  AnomalyStateMachine sm(cfg_);
  EXPECT_EQ(sm.GetState(), AnomalyState::TRACKING);
  EXPECT_DOUBLE_EQ(sm.GetCovarianceScale(), 1.0);
}

TEST_F(AnomalyStateMachineTest, TrackingToDegraded) {
  AnomalyStateMachine sm(cfg_);
  // Feed high chi² values to fill the window and trigger DEGRADED
  for (int i = 0; i < cfg_.chi2_window; ++i) {
    sm.Evaluate(20.0, 0.8, GnssQuality::GOOD, static_cast<double>(i));
  }
  EXPECT_EQ(sm.GetState(), AnomalyState::DEGRADED);
  EXPECT_DOUBLE_EQ(sm.GetCovarianceScale(), 3.0);
}

TEST_F(AnomalyStateMachineTest, DegradedRecovery) {
  AnomalyStateMachine sm(cfg_);
  // Enter DEGRADED
  for (int i = 0; i < cfg_.chi2_window; ++i) {
    sm.Evaluate(20.0, 0.8, GnssQuality::GOOD, static_cast<double>(i));
  }
  ASSERT_EQ(sm.GetState(), AnomalyState::DEGRADED);
  // Feed low chi² to recover
  for (int i = 0; i < cfg_.chi2_window; ++i) {
    sm.Evaluate(2.0, 0.8, GnssQuality::GOOD, 10.0 + i);
  }
  EXPECT_EQ(sm.GetState(), AnomalyState::TRACKING);
}

TEST_F(AnomalyStateMachineTest, Chi2WindowSize) {
  AnomalyStateMachine sm(cfg_);
  // Feed chi2_window-1 high values, then 1 low value
  // The window mean should still be high enough to degrade
  for (int i = 0; i < cfg_.chi2_window - 1; ++i) {
    sm.Evaluate(20.0, 0.8, GnssQuality::GOOD, static_cast<double>(i));
  }
  sm.Evaluate(2.0, 0.8, GnssQuality::GOOD, static_cast<double>(cfg_.chi2_window));
  // Mean = (20*(w-1) + 2)/w = (80+2)/5 = 16.4 > 15 => DEGRADED
  EXPECT_EQ(sm.GetState(), AnomalyState::DEGRADED);
}

TEST_F(AnomalyStateMachineTest, DegradedToLostViaMatchRatio) {
  AnomalyStateMachine sm(cfg_);
  // Enter DEGRADED
  for (int i = 0; i < cfg_.chi2_window; ++i) {
    sm.Evaluate(20.0, 0.8, GnssQuality::GOOD, static_cast<double>(i));
  }
  ASSERT_EQ(sm.GetState(), AnomalyState::DEGRADED);
  // Feed low match ratio for match_lost_duration
  double t = 10.0;
  sm.Evaluate(20.0, 0.1, GnssQuality::GOOD, t);  // start low match timer
  t += cfg_.match_lost_duration + 0.1;
  sm.Evaluate(20.0, 0.1, GnssQuality::GOOD, t);
  EXPECT_EQ(sm.GetState(), AnomalyState::LOST);
  EXPECT_DOUBLE_EQ(sm.GetCovarianceScale(), 10.0);
}

TEST_F(AnomalyStateMachineTest, LostToRelocA) {
  AnomalyStateMachine sm(cfg_);
  // Force to LOST
  sm.ForceState(AnomalyState::LOST);
  ASSERT_EQ(sm.GetState(), AnomalyState::LOST);
  // One Evaluate should transition LOST -> RELOC_A
  sm.Evaluate(20.0, 0.1, GnssQuality::GOOD, 100.0);
  EXPECT_EQ(sm.GetState(), AnomalyState::RELOC_A);
}

TEST_F(AnomalyStateMachineTest, TimeoutRelocAToRelocB) {
  AnomalyStateMachine sm(cfg_);
  sm.ForceState(AnomalyState::LOST);
  sm.Evaluate(20.0, 0.1, GnssQuality::GOOD, 100.0);  // -> RELOC_A
  ASSERT_EQ(sm.GetState(), AnomalyState::RELOC_A);
  // Wait past reloc_a_timeout
  sm.Evaluate(20.0, 0.1, GnssQuality::GOOD, 100.0 + cfg_.reloc_a_timeout + 0.1);
  EXPECT_EQ(sm.GetState(), AnomalyState::RELOC_B);
}

TEST_F(AnomalyStateMachineTest, TimeoutRelocBToLost) {
  AnomalyStateMachine sm(cfg_);
  sm.ForceState(AnomalyState::LOST);
  sm.Evaluate(20.0, 0.1, GnssQuality::GOOD, 100.0);  // -> RELOC_A
  sm.Evaluate(20.0, 0.1, GnssQuality::GOOD, 100.0 + cfg_.reloc_a_timeout + 0.1);  // -> RELOC_B
  ASSERT_EQ(sm.GetState(), AnomalyState::RELOC_B);
  sm.Evaluate(20.0, 0.1, GnssQuality::GOOD, 100.0 + cfg_.reloc_a_timeout + cfg_.reloc_b_timeout + 1.0);
  EXPECT_EQ(sm.GetState(), AnomalyState::LOST);
}

TEST_F(AnomalyStateMachineTest, RelocRecovery) {
  AnomalyStateMachine sm(cfg_);
  sm.ForceState(AnomalyState::LOST);
  sm.Evaluate(20.0, 0.1, GnssQuality::GOOD, 100.0);  // -> RELOC_A
  ASSERT_EQ(sm.GetState(), AnomalyState::RELOC_A);
  // Feed good chi² and match ratio to recover
  for (int i = 0; i < cfg_.chi2_window; ++i) {
    sm.Evaluate(2.0, 0.8, GnssQuality::GOOD, 101.0 + i);
  }
  EXPECT_EQ(sm.GetState(), AnomalyState::TRACKING);
}

TEST_F(AnomalyStateMachineTest, Reset) {
  AnomalyStateMachine sm(cfg_);
  sm.ForceState(AnomalyState::RELOC_B);
  ASSERT_EQ(sm.GetState(), AnomalyState::RELOC_B);
  sm.Reset();
  EXPECT_EQ(sm.GetState(), AnomalyState::TRACKING);
  EXPECT_DOUBLE_EQ(sm.GetCovarianceScale(), 1.0);
}

TEST_F(AnomalyStateMachineTest, ForceState) {
  AnomalyStateMachine sm(cfg_);
  sm.ForceState(AnomalyState::DEGRADED);
  EXPECT_EQ(sm.GetState(), AnomalyState::DEGRADED);
  sm.ForceState(AnomalyState::TRACKING);
  EXPECT_EQ(sm.GetState(), AnomalyState::TRACKING);
}

TEST_F(AnomalyStateMachineTest, CovarianceScaleAllStates) {
  AnomalyStateMachine sm(cfg_);
  sm.ForceState(AnomalyState::TRACKING);
  EXPECT_DOUBLE_EQ(sm.GetCovarianceScale(), 1.0);
  sm.ForceState(AnomalyState::DEGRADED);
  EXPECT_DOUBLE_EQ(sm.GetCovarianceScale(), 3.0);
  sm.ForceState(AnomalyState::LOST);
  EXPECT_DOUBLE_EQ(sm.GetCovarianceScale(), 10.0);
  sm.ForceState(AnomalyState::RELOC_A);
  EXPECT_DOUBLE_EQ(sm.GetCovarianceScale(), 10.0);
  sm.ForceState(AnomalyState::RELOC_B);
  EXPECT_DOUBLE_EQ(sm.GetCovarianceScale(), 10.0);
}

// ═══════════════════════════════════════════════════════════════
// DescriptorRelocator tests
// ═══════════════════════════════════════════════════════════════

class DescriptorRelocatorTest : public ::testing::Test {
 protected:
  RelocConfig cfg_;
  void SetUp() override {
    cfg_.submap_radius = 15.0;
    cfg_.n_sectors = 12;
    cfg_.n_rings = 5;
    cfg_.n_channels = 3;
    cfg_.max_descriptors = 50;
    cfg_.top_k = 3;
    cfg_.ransac_max_iter = 100;
    cfg_.min_inliers = 3;
    cfg_.ransac_inlier_thresh = 1.5;
  }

  std::vector<FgLandmark> makeCircleLandmarks(
      double cx, double cy, double r, int n, uint8_t color = 0) {
    std::vector<FgLandmark> lms;
    for (int i = 0; i < n; ++i) {
      double angle = 2.0 * M_PI * i / n;
      FgLandmark lm;
      lm.id = static_cast<uint32_t>(i);
      lm.x = cx + r * std::cos(angle);
      lm.y = cy + r * std::sin(angle);
      lm.color_type = color;
      lm.obs_count = 5;
      lms.push_back(lm);
    }
    return lms;
  }
};

TEST_F(DescriptorRelocatorTest, BuildDescriptorSize) {
  DescriptorRelocator reloc(cfg_);
  auto lms = makeCircleLandmarks(0, 0, 10.0, 20);
  Pose2 pose{0, 0, 0};
  auto desc = reloc.BuildDescriptor(lms, pose);
  EXPECT_EQ(static_cast<int>(desc.size()),
            cfg_.n_rings * cfg_.n_sectors * cfg_.n_channels);
}

TEST_F(DescriptorRelocatorTest, BuildDescriptorNormalized) {
  DescriptorRelocator reloc(cfg_);
  auto lms = makeCircleLandmarks(0, 0, 10.0, 20);
  Pose2 pose{0, 0, 0};
  auto desc = reloc.BuildDescriptor(lms, pose);
  double norm = 0.0;
  for (float v : desc) norm += v * v;
  EXPECT_NEAR(std::sqrt(norm), 1.0, 1e-5);
}

TEST_F(DescriptorRelocatorTest, EmptyLandmarksZeroDescriptor) {
  DescriptorRelocator reloc(cfg_);
  std::vector<FgLandmark> empty;
  Pose2 pose{0, 0, 0};
  auto desc = reloc.BuildDescriptor(empty, pose);
  double sum = 0.0;
  for (float v : desc) sum += std::abs(v);
  EXPECT_DOUBLE_EQ(sum, 0.0);
}

TEST_F(DescriptorRelocatorTest, DatabaseManagement) {
  DescriptorRelocator reloc(cfg_);
  EXPECT_EQ(reloc.DatabaseSize(), 0u);

  SubMapDescriptor d;
  d.histogram.resize(cfg_.n_rings * cfg_.n_sectors * cfg_.n_channels, 0.1f);
  reloc.AddToDatabase(d);
  EXPECT_EQ(reloc.DatabaseSize(), 1u);

  reloc.ClearDatabase();
  EXPECT_EQ(reloc.DatabaseSize(), 0u);
}

TEST_F(DescriptorRelocatorTest, DatabaseEviction) {
  DescriptorRelocator reloc(cfg_);
  for (int i = 0; i < cfg_.max_descriptors + 10; ++i) {
    SubMapDescriptor d;
    d.keyframe_id = static_cast<uint64_t>(i);
    d.histogram.resize(cfg_.n_rings * cfg_.n_sectors * cfg_.n_channels, 0.1f);
    reloc.AddToDatabase(d);
  }
  EXPECT_EQ(static_cast<int>(reloc.DatabaseSize()), cfg_.max_descriptors);
}

TEST_F(DescriptorRelocatorTest, EmptyDatabaseRelocFails) {
  DescriptorRelocator reloc(cfg_);
  std::vector<float> desc(cfg_.n_rings * cfg_.n_sectors * cfg_.n_channels, 0.1f);
  auto lms = makeCircleLandmarks(0, 0, 10.0, 10);
  Pose2 pose{0, 0, 0};
  auto result = reloc.TryRelocalize(desc, lms, pose);
  EXPECT_FALSE(result.success);
}

TEST_F(DescriptorRelocatorTest, SelfRelocalization) {
  DescriptorRelocator reloc(cfg_);
  auto lms = makeCircleLandmarks(0, 0, 10.0, 20);
  Pose2 pose{0, 0, 0};
  auto desc = reloc.BuildDescriptor(lms, pose);

  SubMapDescriptor db_desc;
  db_desc.keyframe_id = 0;
  db_desc.px = pose.x;
  db_desc.py = pose.y;
  db_desc.ptheta = pose.theta;
  db_desc.histogram = desc;
  for (const auto& lm : lms) {
    db_desc.local_landmark_ids.push_back(lm.id);
    db_desc.local_landmark_positions.emplace_back(lm.x, lm.y);
  }
  reloc.AddToDatabase(db_desc);

  // Relocalize from same position should succeed with near-zero correction
  auto result = reloc.TryRelocalize(desc, lms, pose);
  EXPECT_TRUE(result.success);
  EXPECT_NEAR(result.dx, 0.0, 1.0);
  EXPECT_NEAR(result.dy, 0.0, 1.0);
}

// ═══════════════════════════════════════════════════════════════
// ParticleRelocator tests
// ═══════════════════════════════════════════════════════════════

class ParticleRelocatorTest : public ::testing::Test {
 protected:
  RelocConfig cfg_;
  void SetUp() override {
    cfg_.n_particles = 200;
    cfg_.particle_sigma_xy = 3.0;
    cfg_.particle_sigma_theta = 0.3;
    cfg_.converge_sigma_xy = 0.5;
    cfg_.converge_sigma_theta = 0.1;
  }
};

TEST_F(ParticleRelocatorTest, InitializeSetsParticles) {
  ParticleRelocator reloc(cfg_);
  Pose2 seed{10.0, 20.0, 0.5};
  reloc.Initialize(seed);
  EXPECT_EQ(static_cast<int>(reloc.GetParticles().size()), cfg_.n_particles);
}

TEST_F(ParticleRelocatorTest, ResetClearsParticles) {
  ParticleRelocator reloc(cfg_);
  reloc.Initialize({0, 0, 0});
  EXPECT_GT(reloc.GetParticles().size(), 0u);
  reloc.Reset();
  EXPECT_TRUE(reloc.GetParticles().empty());
  EXPECT_FALSE(reloc.HasConverged());
}

TEST_F(ParticleRelocatorTest, PredictMovesParticles) {
  ParticleRelocator reloc(cfg_);
  Pose2 seed{0, 0, 0};
  reloc.Initialize(seed);

  double mean_x_before = 0;
  for (const auto& p : reloc.GetParticles()) mean_x_before += p.x;
  mean_x_before /= reloc.GetParticles().size();

  // Predict forward at 5 m/s for 1 second
  reloc.Predict(5.0, 0.0, 1.0);

  double mean_x_after = 0;
  for (const auto& p : reloc.GetParticles()) mean_x_after += p.x;
  mean_x_after /= reloc.GetParticles().size();

  EXPECT_GT(mean_x_after, mean_x_before + 3.0);
}

TEST_F(ParticleRelocatorTest, GetEstimateNearSeed) {
  ParticleRelocator reloc(cfg_);
  Pose2 seed{5.0, 10.0, 1.0};
  reloc.Initialize(seed);
  auto est = reloc.GetEstimate();
  EXPECT_NEAR(est.x, seed.x, cfg_.particle_sigma_xy * 2);
  EXPECT_NEAR(est.y, seed.y, cfg_.particle_sigma_xy * 2);
}

TEST_F(ParticleRelocatorTest, GetEstimateUninitializedReturnsZero) {
  ParticleRelocator reloc(cfg_);
  auto est = reloc.GetEstimate();
  EXPECT_DOUBLE_EQ(est.x, 0.0);
  EXPECT_DOUBLE_EQ(est.y, 0.0);
  EXPECT_DOUBLE_EQ(est.theta, 0.0);
}

TEST_F(ParticleRelocatorTest, UpdateConvergesToLandmarks) {
  // Use tight sigma so particles start close to truth
  cfg_.particle_sigma_xy = 0.5;
  cfg_.particle_sigma_theta = 0.05;
  cfg_.n_particles = 500;
  ParticleRelocator reloc(cfg_);

  Pose2 true_pose{0, 0, 0};
  reloc.Initialize(true_pose);

  // Create landmarks in a circle around origin
  std::vector<FgLandmark> landmarks;
  for (int i = 0; i < 8; ++i) {
    double angle = 2.0 * M_PI * i / 8;
    FgLandmark lm;
    lm.id = i;
    lm.x = 5.0 * std::cos(angle);
    lm.y = 5.0 * std::sin(angle);
    landmarks.push_back(lm);
  }

  // Create observations matching the landmarks from true_pose
  std::vector<ConeObservation> obs;
  for (const auto& lm : landmarks) {
    ConeObservation co;
    co.range = std::sqrt(lm.x * lm.x + lm.y * lm.y);
    co.bearing = std::atan2(lm.y, lm.x);
    obs.push_back(co);
  }

  // Run several update cycles
  for (int i = 0; i < 10; ++i) {
    reloc.Update(obs, landmarks);
  }

  auto est = reloc.GetEstimate();
  EXPECT_NEAR(est.x, true_pose.x, 1.0);
  EXPECT_NEAR(est.y, true_pose.y, 1.0);
}

// ═══════════════════════════════════════════════════════════════
// CircleConstraintFactor tests
// ═══════════════════════════════════════════════════════════════

TEST(CircleConstraintFactorTest, PointOnCircleZeroError) {
  gtsam::Point2 c1(0, 0), c2(10, 0);
  double R = 5.0;
  auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  CircleConstraintFactor factor(gtsam::Symbol('l', 0), c1, c2, R, noise);

  // Point exactly on circle 1
  gtsam::Point2 p(5.0, 0.0);
  gtsam::Vector err = factor.evaluateError(p);
  EXPECT_NEAR(err(0), 0.0, 1e-9);
}

TEST(CircleConstraintFactorTest, PointOnCircle2ZeroError) {
  gtsam::Point2 c1(0, 0), c2(10, 0);
  double R = 5.0;
  auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  CircleConstraintFactor factor(gtsam::Symbol('l', 0), c1, c2, R, noise);

  // Point exactly on circle 2
  gtsam::Point2 p(15.0, 0.0);
  gtsam::Vector err = factor.evaluateError(p);
  EXPECT_NEAR(err(0), 0.0, 1e-9);
}

TEST(CircleConstraintFactorTest, PointOutsideCirclePositiveError) {
  gtsam::Point2 c1(0, 0), c2(10, 0);
  double R = 5.0;
  auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  CircleConstraintFactor factor(gtsam::Symbol('l', 0), c1, c2, R, noise);

  // Point at distance 7 from c1 (outside circle 1, closer to c1 than c2)
  gtsam::Point2 p(7.0, 0.0);
  gtsam::Vector err = factor.evaluateError(p);
  // dist to c2 = 3, err2 = |3-5| = 2; dist to c1 = 7, err1 = |7-5| = 2
  // Both equal, uses c1: signed_err = 7-5 = 2
  EXPECT_NEAR(err(0), 2.0, 1e-9);
}

TEST(CircleConstraintFactorTest, PointInsideCircleNegativeError) {
  gtsam::Point2 c1(0, 0), c2(20, 0);
  double R = 5.0;
  auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  CircleConstraintFactor factor(gtsam::Symbol('l', 0), c1, c2, R, noise);

  // Point at distance 3 from c1 (inside circle 1)
  gtsam::Point2 p(3.0, 0.0);
  gtsam::Vector err = factor.evaluateError(p);
  // dist to c1 = 3, err1 = |3-5| = 2; dist to c2 = 17, err2 = |17-5| = 12
  // Uses c1: signed_err = 3-5 = -2
  EXPECT_NEAR(err(0), -2.0, 1e-9);
}

TEST(CircleConstraintFactorTest, JacobianNumericalCheck) {
  gtsam::Point2 c1(0, 0), c2(30, 0);
  double R = 10.0;
  auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  CircleConstraintFactor factor(gtsam::Symbol('l', 0), c1, c2, R, noise);

  // Test at a point clearly closer to c1 (not on circle, not at center)
  gtsam::Point2 p(8.0, 6.0);  // dist to c1 = 10, on circle => err=0

  // Test at a point off-circle
  gtsam::Point2 p2(6.0, 4.0);  // dist to c1 = sqrt(52) ~7.2
  gtsam::Matrix H_analytical;
  factor.evaluateError(p2, H_analytical);

  auto numerical_H = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Point2>(
      [&](const gtsam::Point2& pt) { return factor.evaluateError(pt); }, p2);

  EXPECT_TRUE(H_analytical.isApprox(numerical_H, 1e-5))
      << "Analytical:\n" << H_analytical << "\nNumerical:\n" << numerical_H;
}

TEST(CircleConstraintFactorTest, Clone) {
  gtsam::Point2 c1(0, 0), c2(10, 0);
  auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  CircleConstraintFactor factor(gtsam::Symbol('l', 0), c1, c2, 5.0, noise);
  auto cloned = factor.clone();
  EXPECT_NE(cloned, nullptr);
}

// ═══════════════════════════════════════════════════════════════
// LocationMapper tests (geometry robustness)
// ═══════════════════════════════════════════════════════════════

class LocationMapperTest : public ::testing::Test {
 protected:
  LocationParams params_;
  void SetUp() override {
    params_.merge_distance = 2.5;
    params_.max_map_size = 500;
    params_.min_obs_to_keep = 1;
    params_.local_cone_range = 50.0;
    params_.min_confidence_to_add = 0.0;
    params_.min_confidence_to_merge = 0.0;
    params_.max_cone_height = 1.0;
    params_.max_cone_width = 1.0;
    params_.min_cone_height = 0.01;
  }

  // Helper: create a CarState at origin and feed it to mapper
  void initMapper(LocationMapper &mapper) {
    CarState cs;
    cs.car_state.x = 0.0;
    cs.car_state.y = 0.0;
    cs.car_state.theta = 0.0;
    cs.V = 0.0;
    mapper.UpdateFromCarState(cs);
  }

  // Helper: create detections along X axis at given positions
  ConeDetections makeDetections(const std::vector<double> &x_positions, double y = 0.0) {
    ConeDetections dets;
    for (double x : x_positions) {
      ConeDetection d;
      d.point.x = x;
      d.point.y = y;
      d.point.z = 0.15;
      d.bbox_min = {x - 0.1, y - 0.1, 0.0};
      d.bbox_max = {x + 0.1, y + 0.1, 0.3};
      d.confidence = 0.5;
      d.color_type = 4;  // NONE
      dets.detections.push_back(d);
    }
    return dets;
  }
};

TEST_F(LocationMapperTest, ShortPathSuppression_FewCones_ClearsOutput) {
  params_.short_path_suppression.enabled = true;
  params_.short_path_suppression.min_cone_count = 3;
  params_.short_path_suppression.min_path_length = 3.0;
  params_.short_path_suppression.reject_single_cone_paths = true;

  LocationMapper mapper(params_);
  initMapper(mapper);

  // 2 cones close together (< min_cone_count)
  auto dets = makeDetections({3.0, 3.5});
  ConeMap map;
  PointCloudPtr cloud;
  mapper.UpdateFromCones(dets, &map, &cloud);

  EXPECT_TRUE(map.cones.empty());
}

TEST_F(LocationMapperTest, ShortPathSuppression_EnoughCones_Passes) {
  params_.short_path_suppression.enabled = true;
  params_.short_path_suppression.min_cone_count = 3;
  params_.short_path_suppression.min_path_length = 3.0;
  params_.short_path_suppression.reject_single_cone_paths = true;

  LocationMapper mapper(params_);
  initMapper(mapper);

  // 5 cones spaced 5m apart (total span = 20m)
  auto dets = makeDetections({5.0, 10.0, 15.0, 20.0, 25.0});
  ConeMap map;
  PointCloudPtr cloud;
  mapper.UpdateFromCones(dets, &map, &cloud);

  EXPECT_GE(static_cast<int>(map.cones.size()), 3);
}

TEST_F(LocationMapperTest, MissingConeFallback_InterpolatesGap) {
  params_.missing_cone_fallback.enabled = true;
  params_.missing_cone_fallback.expected_spacing = 5.0;
  params_.missing_cone_fallback.max_interpolation_distance = 20.0;
  params_.missing_cone_fallback.min_confidence_for_interpolation = 0.15;
  params_.missing_cone_fallback.max_consecutive_missing = 3;

  LocationMapper mapper(params_);
  initMapper(mapper);

  // 2 cones 15m apart (gap = 3x expected_spacing, should interpolate)
  auto dets = makeDetections({5.0, 20.0});
  ConeMap map;
  PointCloudPtr cloud;
  mapper.UpdateFromCones(dets, &map, &cloud);

  // Should have original 2 + interpolated cones
  EXPECT_GT(static_cast<int>(map.cones.size()), 2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
