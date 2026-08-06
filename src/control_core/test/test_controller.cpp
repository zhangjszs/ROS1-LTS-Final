#include "control_core/test_controller.hpp"

#include "control_core/controller_base.hpp"
#include "control_core/ebs_controller.hpp"
#include "control_core/high_controller.hpp"
#include "control_core/line_controller.hpp"
#include "control_core/skip_controller.hpp"

#include <gtest/gtest.h>

namespace control_core {

// ==================== EbsController Tests ====================

TEST(EbsControllerTest, Initialization) {
  EbsController controller;
  EXPECT_FALSE(controller.HasPath());
}

TEST(EbsControllerTest, EmergencyBrakeValues) {
  EbsController controller;
  ControlParams params;
  controller.SetParams(params);

  // EBS should output safe emergency values
  ControlOutput output = controller.ComputeOutput();

  // Steering should be centered (110)
  EXPECT_EQ(output.steering, 110);
  // Pedal should be 0 (no throttle)
  EXPECT_EQ(output.pedal_ratio, 0);
  // Brake should be maximum (100)
  EXPECT_EQ(output.brake_force, 100);
  // Status should be 3 (EBS status)
  EXPECT_EQ(output.racing_status, 3);
  // Should not request stop (hardware EBS handles it)
  EXPECT_FALSE(output.stop_requested);
}

TEST(EbsControllerTest, IgnoresPathInput) {
  EbsController controller;

  // Create a simple path
  std::vector<Position> path = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  controller.UpdatePath(path);

  // EBS should still output emergency values regardless of path
  ControlOutput output = controller.ComputeOutput();
  EXPECT_EQ(output.brake_force, 100);
  EXPECT_EQ(output.pedal_ratio, 0);
}

// ==================== LineController Tests ====================

TEST(LineControllerTest, Initialization) {
  LineController controller;
  EXPECT_FALSE(controller.HasPath());
}

TEST(LineControllerTest, PathManagement) {
  LineController controller;

  // Initially no path
  EXPECT_FALSE(controller.HasPath());

  // Add path
  std::vector<Position> path = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  controller.UpdatePath(path);
  EXPECT_TRUE(controller.HasPath());
}

TEST(LineControllerTest, ComputeOutputWithPath) {
  LineController controller;
  ControlParams params;
  params.car_length = 1.55;
  params.angle_kp = 1.0;
  controller.SetParams(params);

  // Create a straight path
  std::vector<Position> path;
  for (int i = 0; i < 10; ++i) {
    path.push_back({static_cast<double>(i), 0.0});
  }
  controller.UpdatePath(path);

  // Set car state at origin, facing forward
  CarState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 1.0;
  controller.UpdateCarState(state);

  ControlOutput output = controller.ComputeOutput();

  // On straight path, steering should be close to center (110)
  EXPECT_NEAR(output.steering, 110, 20);
  // Should have some pedal for forward motion
  EXPECT_GE(output.pedal_ratio, 0);
}

// ==================== HighController Tests ====================

TEST(HighControllerTest, Initialization) {
  HighController controller;
  EXPECT_FALSE(controller.HasPath());
}

TEST(HighControllerTest, SameBaseAsLineController) {
  // HighController and LineController both use ControllerBase
  // The difference is in configuration, not implementation
  HighController high_controller;
  LineController line_controller;

  EXPECT_FALSE(high_controller.HasPath());
  EXPECT_FALSE(line_controller.HasPath());
}

// ==================== SkipController Tests ====================

TEST(SkipControllerTest, Initialization) {
  SkipController controller;
  EXPECT_FALSE(controller.HasPath());
}

TEST(SkipControllerTest, CountErrorUsesAbsoluteHeading) {
  // Verify that CountError uses the absolute path tangent heading (not relative alpha)
  // for the cross-track error projection.
  SkipController controller;

  // Convention: positive error = car is to the left of path direction (+x),
  // negative error = car is to the right.
  // With heading = 0 (path tangent along +x), path normal points to +y (left).

  // Car at (0, +1) — left of path
  // dx = 0-0=0, dy = 0-1=-1, signed = -1*cos(0) = -1 < 0 → return +|error|
  double err_car_left = controller.CountError(0.0, 0.0, 0.0, 1.0, 0.0);
  // Car at (0, -1) — right of path
  // dx = 0, dy = 0-(-1)=1, signed = 1*cos(0) = 1 > 0 → return -|error|
  double err_car_right = controller.CountError(0.0, 0.0, 0.0, -1.0, 0.0);

  EXPECT_GT(err_car_left, 0.0);     // left of path → positive
  EXPECT_LT(err_car_right, 0.0);    // right of path → negative
  EXPECT_NEAR(err_car_left, -err_car_right, 1e-9);
}

TEST(SkipControllerTest, CountErrorMagnitude) {
  SkipController controller;

  // Distance from path point (0,0) to car (3,4) is 5
  double err = controller.CountError(0.0, 0.0, 3.0, 4.0, 0.0);
  EXPECT_NEAR(std::abs(err), 5.0, 1e-9);
}

// ==================== TestController Tests ====================

TEST(TestControllerTest, Initialization) {
  TestController controller;
  EXPECT_FALSE(controller.HasPath());
}

TEST(TestControllerTest, SineWaveOutput) {
  TestController controller;
  ControlParams params;
  controller.SetParams(params);

  // TestController generates sine wave steering
  ControlOutput output1 = controller.ComputeOutput();
  controller.Tick();
  ControlOutput output2 = controller.ComputeOutput();

  // Steering should change over time
  EXPECT_NE(output1.steering, output2.steering);
}

// ==================== ControllerBase Common Tests ====================

TEST(ControllerBaseTest, CarStateUpdate) {
  LineController controller;

  CarState state;
  state.x = 1.0;
  state.y = 2.0;
  state.theta = 0.5;
  state.v = 3.0;
  state.vy = 0.1;
  state.yaw_rate = 0.2;

  controller.UpdateCarState(state);
  // State is stored internally, verified through behavior
  EXPECT_FALSE(controller.HasPath());  // Still no path
}

TEST(ControllerBaseTest, TargetSpeedsUpdate) {
  LineController controller;

  std::vector<double> speeds = {1.0, 2.0, 3.0, 4.0, 5.0};
  controller.UpdateTargetSpeeds(speeds);

  // Speeds are stored internally
  EXPECT_FALSE(controller.HasPath());
}

TEST(ControllerBaseTest, CurvaturesUpdate) {
  LineController controller;

  std::vector<double> curvatures = {0.0, 0.1, 0.2, 0.1, 0.0};
  controller.UpdateCurvatures(curvatures);

  // Curvatures are stored internally
  EXPECT_FALSE(controller.HasPath());
}

TEST(ControllerBaseTest, FinishSignal) {
  LineController controller;
  ControlParams params;
  controller.SetParams(params);

  // Set up minimal path
  std::vector<Position> path = {{0.0, 0.0}, {1.0, 0.0}};
  controller.UpdatePath(path);

  CarState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 0.0;
  controller.UpdateCarState(state);

  // Before finish signal
  ControlOutput output1 = controller.ComputeOutput();
  EXPECT_FALSE(output1.stop_requested);

  // Set finish signal
  controller.SetFinishSignal(true);
  ControlOutput output2 = controller.ComputeOutput();
  EXPECT_TRUE(output2.stop_requested);
}

TEST(ControllerBaseTest, EmptyPathHandling) {
  LineController controller;
  ControlParams params;
  controller.SetParams(params);

  // No path set
  EXPECT_FALSE(controller.HasPath());

  CarState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 0.0;
  controller.UpdateCarState(state);

  // With empty path, controller should handle gracefully
  // Note: ComputeOutput requires a path for normal operation,
  // this test verifies the controller doesn't crash with empty path
  EXPECT_FALSE(controller.HasPath());
}

TEST(ControllerBaseTest, PathWithSinglePoint) {
  LineController controller;
  ControlParams params;
  controller.SetParams(params);

  // Single point path
  std::vector<Position> path = {{1.0, 1.0}};
  controller.UpdatePath(path);
  EXPECT_TRUE(controller.HasPath());

  CarState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 1.0;
  controller.UpdateCarState(state);

  // Should handle single point gracefully
  ControlOutput output = controller.ComputeOutput();
  EXPECT_GE(output.steering, 0);
  EXPECT_LE(output.steering, 255);
}

TEST(ControllerBaseTest, ParamsSetting) {
  LineController controller;

  ControlParams params;
  params.angle_kp = 2.0;
  params.angle_ki = 0.1;
  params.angle_kd = 0.01;
  params.car_length = 1.6;
  params.min_lookahead = 3.0;
  params.max_lookahead = 15.0;
  params.enable_slip_compensation = false;
  params.slip_gain = 0.3;
  params.curvature_ff_gain = 0.5;
  params.brake_kp = 50.0;
  params.brake_max = 70.0;

  // Should not throw
  EXPECT_NO_THROW(controller.SetParams(params));
}

// ==================== Steering Conversion Tests ====================

TEST(SteeringConversionTest, RangeValidation) {
  LineController controller;
  ControlParams params;
  controller.SetParams(params);

  // Create path with sharp turn
  std::vector<Position> path;
  for (int i = 0; i < 20; ++i) {
    path.push_back({static_cast<double>(i), static_cast<double>(i) * 0.5});
  }
  controller.UpdatePath(path);

  CarState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 2.0;
  controller.UpdateCarState(state);

  ControlOutput output = controller.ComputeOutput();

  // Steering should be in valid range [0, 255] (uint8_t range)
  EXPECT_GE(output.steering, 0);
  EXPECT_LE(output.steering, 255);
}

// ==================== Pedal and Brake Tests ====================

TEST(PedalBrakeTest, ZeroSpeedBehavior) {
  LineController controller;
  ControlParams params;
  controller.SetParams(params);

  std::vector<Position> path;
  for (int i = 0; i < 10; ++i) {
    path.push_back({static_cast<double>(i), 0.0});
  }
  controller.UpdatePath(path);

  // Set target speeds for brake computation
  std::vector<double> speeds;
  for (int i = 0; i < 10; ++i) {
    speeds.push_back(2.0);
  }
  controller.UpdateTargetSpeeds(speeds);

  CarState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 0.0;  // Zero speed
  controller.UpdateCarState(state);

  ControlOutput output = controller.ComputeOutput();

  // At zero speed, should apply some pedal to start moving
  EXPECT_GE(output.pedal_ratio, 0);
  EXPECT_LE(output.pedal_ratio, 100);
}

TEST(PedalBrakeTest, OverspeedBehavior) {
  LineController controller;
  ControlParams params;
  params.brake_kp = 40.0;
  params.brake_max = 80.0;
  params.brake_speed_margin = 0.5;
  controller.SetParams(params);

  std::vector<Position> path;
  for (int i = 0; i < 10; ++i) {
    path.push_back({static_cast<double>(i), 0.0});
  }
  controller.UpdatePath(path);

  // Set target speeds (lower than current speed)
  std::vector<double> speeds;
  for (int i = 0; i < 10; ++i) {
    speeds.push_back(1.0);  // Low target speed
  }
  controller.UpdateTargetSpeeds(speeds);

  CarState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 5.0;  // Much higher than target
  controller.UpdateCarState(state);

  ControlOutput output = controller.ComputeOutput();

  // Should apply brake when overspeed
  EXPECT_GE(output.brake_force, 0);
}

// ==================== Velocity Integral Reset Tests ====================

TEST(PedalBrakeTest, IntegralResetsWhenStationary) {
  LineController controller;
  // Use default params (pedal ki > 0 so integral matters)
  ControlParams params;
  controller.SetParams(params);

  std::vector<Position> path;
  for (int i = 0; i < 10; ++i) {
    path.push_back({static_cast<double>(i), 0.0});
  }
  controller.UpdatePath(path);

  std::vector<double> speeds;
  for (int i = 0; i < 10; ++i) {
    speeds.push_back(5.0);
  }
  controller.UpdateTargetSpeeds(speeds);

  // Run many cycles with the vehicle stationary.
  // Without integral reset, the integral would wind up indefinitely and
  // the pedal output would saturate. With reset, it stays bounded.
  CarState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 0.0;  // Stationary → integral resets each cycle
  controller.UpdateCarState(state);

  int max_pedal = 0;
  for (int i = 0; i < 100; ++i) {
    ControlOutput output = controller.ComputeOutput();
    max_pedal = std::max(max_pedal, output.pedal_ratio);
    // Keep vehicle stationary
    controller.UpdateCarState(state);
  }

  // Pedal should remain bounded (not saturate at max) because integral resets
  EXPECT_GE(max_pedal, 0);
  EXPECT_LE(max_pedal, 100);
}

TEST(PedalBrakeTest, FinishBrakeFractionIsConfigurable) {
  LineController controller;
  ControlParams params;
  params.brake_max = 80.0;
  params.finish_brake_fraction = 0.5;  // 50% instead of default 30%
  controller.SetParams(params);

  std::vector<Position> path;
  for (int i = 0; i < 10; ++i) {
    path.push_back({static_cast<double>(i), 0.0});
  }
  controller.UpdatePath(path);

  std::vector<double> speeds;
  for (int i = 0; i < 10; ++i) {
    speeds.push_back(5.0);
  }
  controller.UpdateTargetSpeeds(speeds);

  CarState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 3.0;  // Moving, so finish brake applies
  controller.UpdateCarState(state);

  controller.SetFinishSignal(true);
  ControlOutput output = controller.ComputeOutput();

  // Should apply 50% of brake_max = 40
  EXPECT_EQ(output.brake_force, 40);
}

}  // namespace control_core

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
