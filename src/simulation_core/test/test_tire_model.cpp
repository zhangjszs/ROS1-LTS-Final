#include <gtest/gtest.h>
#include "simulation_core/tire_model.hpp"
#include <cmath>

using namespace simulation_core;

class TireModelTest : public ::testing::Test {
protected:
    TireModel tire;
    TireParams params;

    void SetUp() override {
        params.B = 10.0;
        params.C = 1.65;
        params.D = 1.0;
        params.E = -0.5;
        params.friction = 1.0;
        tire.setParams(params);
    }
};

TEST_F(TireModelTest, ZeroSlipZeroForce) {
    double Fx, Fy;
    tire.calculateForces(0.0, 0.0, 1000.0, Fx, Fy);
    EXPECT_NEAR(Fx, 0.0, 1e-6);
    EXPECT_NEAR(Fy, 0.0, 1e-6);
}

TEST_F(TireModelTest, ZeroNormalLoadZeroForce) {
    double Fx, Fy;
    tire.calculateForces(0.1, 0.1, 0.0, Fx, Fy);
    EXPECT_NEAR(Fx, 0.0, 1e-6);
    EXPECT_NEAR(Fy, 0.0, 1e-6);
}

TEST_F(TireModelTest, PositiveSlipRatioPositiveFx) {
    double Fx = tire.calculateFx(0.1, 1000.0);
    EXPECT_GT(Fx, 0.0);
}

TEST_F(TireModelTest, NegativeSlipRatioNegativeFx) {
    double Fx = tire.calculateFx(-0.1, 1000.0);
    EXPECT_LT(Fx, 0.0);
}

TEST_F(TireModelTest, PositiveSlipAnglePositiveFy) {
    double Fy = tire.calculateFy(0.1, 1000.0);
    EXPECT_GT(Fy, 0.0);
}

TEST_F(TireModelTest, NegativeSlipAngleNegativeFy) {
    double Fy = tire.calculateFy(-0.1, 1000.0);
    EXPECT_LT(Fy, 0.0);
}

TEST_F(TireModelTest, ForceScalesWithNormalLoad) {
    double Fx1 = tire.calculateFx(0.1, 1000.0);
    double Fx2 = tire.calculateFx(0.1, 2000.0);
    EXPECT_NEAR(Fx2 / Fx1, 2.0, 0.1);
}

TEST_F(TireModelTest, FrictionCircleLimit) {
    double Fx, Fy;
    // High slip ratio and angle should be limited by friction circle
    tire.calculateForces(0.5, 0.5, 1000.0, Fx, Fy);
    double F_total = std::hypot(Fx, Fy);
    double F_max = params.friction * 1000.0 * params.D;
    EXPECT_LE(F_total, F_max + 1e-6);
}

TEST_F(TireModelTest, PeakForceAtOptimalSlip) {
    // Magic formula should have a peak around slip = 0.1-0.2
    double Fx_low = std::abs(tire.calculateFx(0.05, 1000.0));
    double Fx_peak = std::abs(tire.calculateFx(0.15, 1000.0));
    double Fx_high = std::abs(tire.calculateFx(0.5, 1000.0));

    EXPECT_GT(Fx_peak, Fx_low);
    EXPECT_GT(Fx_peak, Fx_high);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
