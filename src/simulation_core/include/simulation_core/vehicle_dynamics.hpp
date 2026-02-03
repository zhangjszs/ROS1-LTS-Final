#pragma once

#include "simulation_core/types.hpp"
#include "simulation_core/tire_model.hpp"
#include <array>

namespace simulation_core {

/**
 * @brief Vehicle dynamics simulation using bicycle model with tire forces
 *
 * Implements a 6-DOF vehicle model with:
 * - Pacejka tire model for force calculation
 * - Load transfer (longitudinal and lateral)
 * - Aerodynamic forces (drag and downforce)
 * - RK4 integration for numerical stability
 */
class VehicleDynamics {
public:
    VehicleDynamics();
    explicit VehicleDynamics(const VehicleParams& vehicle_params,
                             const TireParams& tire_params);

    /**
     * @brief Set vehicle parameters
     */
    void setVehicleParams(const VehicleParams& params);

    /**
     * @brief Set tire parameters
     */
    void setTireParams(const TireParams& params);

    /**
     * @brief Reset simulation to initial state
     * @param initial Initial vehicle state
     */
    void reset(const VehicleState& initial);

    /**
     * @brief Reset to track start position
     * @param track Track definition with start position
     */
    void reset(const Track& track);

    /**
     * @brief Advance simulation by one time step
     * @param input Control input (steering, throttle, brake)
     * @param dt Time step [s]
     */
    void step(const ControlInput& input, double dt);

    /**
     * @brief Get current simulation state
     */
    const SimulationState& state() const { return state_; }

    /**
     * @brief Get vehicle parameters
     */
    const VehicleParams& vehicleParams() const { return vehicle_params_; }

    /**
     * @brief Get tire model
     */
    const TireModel& tireModel() const { return tire_model_; }

private:
    /**
     * @brief Calculate normal loads on each wheel with load transfer
     */
    void calculateNormalLoads();

    /**
     * @brief Calculate aerodynamic forces
     * @param Fx_aero Output drag force [N]
     * @param Fz_aero Output downforce [N]
     */
    void calculateAeroForces(double& Fx_aero, double& Fz_aero) const;

    /**
     * @brief Calculate slip ratios and angles for each wheel
     */
    void calculateSlips();

    /**
     * @brief Calculate tire forces for all wheels
     * @param Fx_total Output total longitudinal force [N]
     * @param Fy_total Output total lateral force [N]
     * @param Mz_total Output total yaw moment [Nm]
     */
    void calculateTireForces(double& Fx_total, double& Fy_total, double& Mz_total);

    /**
     * @brief State derivative for integration
     * @param state Current state
     * @param input Control input
     * @return State derivatives [dx, dy, dyaw, dvx, dvy, dyaw_rate]
     */
    std::array<double, 6> stateDerivative(const VehicleState& state,
                                           const ControlInput& input);

    /**
     * @brief RK4 integration step
     * @param input Control input
     * @param dt Time step [s]
     */
    void integrateRK4(const ControlInput& input, double dt);

    /**
     * @brief FSSIM-style kinematic correction for low-speed stability
     * Blends kinematic and dynamic models based on velocity
     * @param x_dyn Dynamic model state
     * @param x_prev Previous state
     * @param input Control input
     * @param dt Time step [s]
     * @return Corrected state
     */
    VehicleState kinematicCorrection(const VehicleState& x_dyn,
                                      const VehicleState& x_prev,
                                      const ControlInput& input,
                                      double dt);

    /**
     * @brief Apply steering dynamics (rate limiting)
     * @param target Target steering angle [rad]
     * @param dt Time step [s]
     */
    void applySteeringDynamics(double target, double dt);

    VehicleParams vehicle_params_;
    TireModel tire_model_;
    SimulationState state_;

    // Intermediate calculations
    double total_downforce_ = 0.0;
    std::array<double, 4> tire_Fx_;
    std::array<double, 4> tire_Fy_;

    // Steering dynamics
    double steering_rate_limit_ = 1.0;  // rad/s
};

}  // namespace simulation_core
