#include <gtest/gtest.h>
#include "simulation_core/vehicle_dynamics.hpp"
#include <cmath>

using namespace simulation_core;

class VehicleDynamicsTest : public ::testing::Test {
protected:
    VehicleDynamics dynamics;
    VehicleParams vehicle_params;
    TireParams tire_params;

    void SetUp() override {
        vehicle_params.mass = 250.0;
        vehicle_params.wheelbase = 1.55;
        vehicle_params.track_width = 1.2;
        vehicle_params.cg_height = 0.3;
        vehicle_params.cg_to_front = 0.775;
        vehicle_params.cg_to_rear = 0.775;
        vehicle_params.inertia_z = 150.0;
        vehicle_params.max_steering = 0.5;
        vehicle_params.max_torque = 200.0;
        vehicle_params.max_brake_torque = 500.0;

        tire_params.B = 10.0;
        tire_params.C = 1.65;
        tire_params.D = 1.0;
        tire_params.E = -0.5;
        tire_params.friction = 1.0;

        dynamics.setVehicleParams(vehicle_params);
        dynamics.setTireParams(tire_params);

        VehicleState initial;
        initial.x = 0.0;
        initial.y = 0.0;
        initial.yaw = 0.0;
        initial.vx = 5.0;  // Start with some velocity
        dynamics.reset(initial);
    }
};

TEST_F(VehicleDynamicsTest, InitialStateCorrect) {
    const auto& state = dynamics.state();
    EXPECT_DOUBLE_EQ(state.vehicle.x, 0.0);
    EXPECT_DOUBLE_EQ(state.vehicle.y, 0.0);
    EXPECT_DOUBLE_EQ(state.vehicle.yaw, 0.0);
    EXPECT_DOUBLE_EQ(state.vehicle.vx, 5.0);
    EXPECT_DOUBLE_EQ(state.sim_time, 0.0);
}

TEST_F(VehicleDynamicsTest, TimeAdvances) {
    ControlInput input;
    dynamics.step(input, 0.01);
    EXPECT_NEAR(dynamics.state().sim_time, 0.01, 1e-9);
}

TEST_F(VehicleDynamicsTest, VehicleMovesForward) {
    ControlInput input;
    input.throttle = 0.5;

    double initial_x = dynamics.state().vehicle.x;
    for (int i = 0; i < 100; ++i) {
        dynamics.step(input, 0.01);
    }

    EXPECT_GT(dynamics.state().vehicle.x, initial_x);
}

TEST_F(VehicleDynamicsTest, SteeringCausesYawChange) {
    ControlInput input;
    input.steering = 0.2;
    input.throttle = 0.3;

    double initial_yaw = dynamics.state().vehicle.yaw;
    for (int i = 0; i < 100; ++i) {
        dynamics.step(input, 0.01);
    }

    EXPECT_NE(dynamics.state().vehicle.yaw, initial_yaw);
}

TEST_F(VehicleDynamicsTest, BrakingReducesVelocity) {
    ControlInput input;
    input.brake = 0.5;

    double initial_vx = dynamics.state().vehicle.vx;
    for (int i = 0; i < 100; ++i) {
        dynamics.step(input, 0.01);
    }

    EXPECT_LT(dynamics.state().vehicle.vx, initial_vx);
}

TEST_F(VehicleDynamicsTest, SteeringIsRateLimited) {
    ControlInput input;
    input.steering = 0.5;  // Max steering

    dynamics.step(input, 0.01);

    // Steering should not jump to max immediately
    EXPECT_LT(std::abs(dynamics.state().vehicle.steering), 0.5);
}

TEST_F(VehicleDynamicsTest, NormalLoadsArePositive) {
    ControlInput input;
    dynamics.step(input, 0.01);

    const auto& state = dynamics.state();
    for (int i = 0; i < 4; ++i) {
        EXPECT_GE(state.normal_loads[i], 0.0);
    }
}

TEST_F(VehicleDynamicsTest, ResetFromTrack) {
    Track track;
    track.start_x = 10.0;
    track.start_y = 5.0;
    track.start_heading = M_PI / 4.0;

    dynamics.reset(track);

    const auto& state = dynamics.state();
    EXPECT_DOUBLE_EQ(state.vehicle.x, 10.0);
    EXPECT_DOUBLE_EQ(state.vehicle.y, 5.0);
    EXPECT_DOUBLE_EQ(state.vehicle.yaw, M_PI / 4.0);
    EXPECT_DOUBLE_EQ(state.sim_time, 0.0);
}

TEST_F(VehicleDynamicsTest, IntegrationStability) {
    // Run simulation for many steps to check stability
    ControlInput input;
    input.throttle = 0.3;
    input.steering = 0.1;

    for (int i = 0; i < 10000; ++i) {
        dynamics.step(input, 0.001);
    }

    const auto& state = dynamics.state();
    // Check that values are finite
    EXPECT_TRUE(std::isfinite(state.vehicle.x));
    EXPECT_TRUE(std::isfinite(state.vehicle.y));
    EXPECT_TRUE(std::isfinite(state.vehicle.vx));
    EXPECT_TRUE(std::isfinite(state.vehicle.vy));
    EXPECT_TRUE(std::isfinite(state.vehicle.yaw));
    EXPECT_TRUE(std::isfinite(state.vehicle.yaw_rate));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
