#include "simulation_core/vehicle_dynamics.hpp"
#include <cmath>
#include <algorithm>

namespace simulation_core {

VehicleDynamics::VehicleDynamics()
    : vehicle_params_(), tire_model_(), state_() {
    tire_Fx_.fill(0.0);
    tire_Fy_.fill(0.0);
}

VehicleDynamics::VehicleDynamics(const VehicleParams& vehicle_params,
                                   const TireParams& tire_params)
    : vehicle_params_(vehicle_params), tire_model_(tire_params), state_() {
    tire_Fx_.fill(0.0);
    tire_Fy_.fill(0.0);
}

void VehicleDynamics::setVehicleParams(const VehicleParams& params) {
    vehicle_params_ = params;
}

void VehicleDynamics::setTireParams(const TireParams& params) {
    tire_model_.setParams(params);
}

void VehicleDynamics::reset(const VehicleState& initial) {
    state_.vehicle = initial;
    state_.input = ControlInput();
    state_.sim_time = 0.0;
    for (int i = 0; i < 4; ++i) {
        state_.wheel_speeds[i] = 0.0;
        state_.slip_ratios[i] = 0.0;
        state_.slip_angles[i] = 0.0;
        state_.normal_loads[i] = vehicle_params_.mass * 9.81 / 4.0;
    }
    tire_Fx_.fill(0.0);
    tire_Fy_.fill(0.0);
    total_downforce_ = 0.0;
}

void VehicleDynamics::reset(const Track& track) {
    VehicleState initial;
    initial.x = track.start_x;
    initial.y = track.start_y;
    initial.yaw = track.start_heading;
    reset(initial);
}

void VehicleDynamics::step(const ControlInput& input, double dt) {
    // Apply steering dynamics
    applySteeringDynamics(input.steering, dt);

    // Store input
    state_.input = input;
    state_.input.steering = state_.vehicle.steering;

    // Calculate loads and forces
    calculateNormalLoads();
    calculateSlips();

    // Integrate dynamics
    integrateRK4(state_.input, dt);

    // Update simulation time
    state_.sim_time += dt;
}

void VehicleDynamics::calculateNormalLoads() {
    const double g = 9.81;
    const double m = vehicle_params_.mass;
    const double L = vehicle_params_.wheelbase;
    const double lf = vehicle_params_.cg_to_front;
    const double lr = vehicle_params_.cg_to_rear;
    const double h = vehicle_params_.cg_height;
    const double tw = vehicle_params_.track_width;

    // Aerodynamic downforce
    double Fx_aero, Fz_aero;
    calculateAeroForces(Fx_aero, Fz_aero);
    total_downforce_ = Fz_aero;

    // Static weight distribution
    double Fz_front_static = m * g * lr / L;
    double Fz_rear_static = m * g * lf / L;

    // Add downforce (assume 50/50 distribution)
    Fz_front_static += Fz_aero * 0.5;
    Fz_rear_static += Fz_aero * 0.5;

    // Longitudinal load transfer
    double ax = 0.0;  // Will be calculated from tire forces
    // For now, use a simple approximation based on throttle/brake
    if (state_.input.throttle > 0.01) {
        ax = state_.input.throttle * vehicle_params_.max_torque /
             vehicle_params_.wheel_radius / m;
    } else if (state_.input.brake > 0.01) {
        ax = -state_.input.brake * vehicle_params_.max_brake_torque /
             vehicle_params_.wheel_radius / m;
    }
    double dFz_long = m * ax * h / L;

    // Lateral load transfer
    double ay = state_.vehicle.vx * state_.vehicle.yaw_rate;
    double dFz_lat_front = m * ay * h * lr / (L * tw);
    double dFz_lat_rear = m * ay * h * lf / (L * tw);

    // Calculate individual wheel loads
    // FL, FR, RL, RR
    double Fz_front = Fz_front_static - dFz_long;
    double Fz_rear = Fz_rear_static + dFz_long;

    state_.normal_loads[0] = std::max(0.0, (Fz_front / 2.0) - dFz_lat_front);  // FL
    state_.normal_loads[1] = std::max(0.0, (Fz_front / 2.0) + dFz_lat_front);  // FR
    state_.normal_loads[2] = std::max(0.0, (Fz_rear / 2.0) - dFz_lat_rear);    // RL
    state_.normal_loads[3] = std::max(0.0, (Fz_rear / 2.0) + dFz_lat_rear);    // RR
}

void VehicleDynamics::calculateAeroForces(double& Fx_aero, double& Fz_aero) const {
    double v = std::hypot(state_.vehicle.vx, state_.vehicle.vy);
    double q = 0.5 * vehicle_params_.air_density * v * v;
    double A = vehicle_params_.frontal_area;

    Fx_aero = -vehicle_params_.drag_coeff * q * A;  // Drag (negative = opposing motion)
    Fz_aero = vehicle_params_.downforce_coeff * q * A;  // Downforce (positive = into ground)
}

void VehicleDynamics::calculateSlips() {
    const double vx = state_.vehicle.vx;
    const double vy = state_.vehicle.vy;
    const double r = state_.vehicle.yaw_rate;
    const double delta = state_.vehicle.steering;
    const double lf = vehicle_params_.cg_to_front;
    const double lr = vehicle_params_.cg_to_rear;
    const double tw = vehicle_params_.track_width / 2.0;

    // Wheel velocities in vehicle frame
    // Front wheels
    double vx_fl = vx - r * tw;
    double vy_fl = vy + r * lf;
    double vx_fr = vx + r * tw;
    double vy_fr = vy + r * lf;

    // Rear wheels
    double vx_rl = vx - r * tw;
    double vy_rl = vy - r * lr;
    double vx_rr = vx + r * tw;
    double vy_rr = vy - r * lr;

    // Transform front wheel velocities to wheel frame (rotated by steering angle)
    double cos_d = std::cos(delta);
    double sin_d = std::sin(delta);

    double vx_fl_wheel = vx_fl * cos_d + vy_fl * sin_d;
    double vy_fl_wheel = -vx_fl * sin_d + vy_fl * cos_d;
    double vx_fr_wheel = vx_fr * cos_d + vy_fr * sin_d;
    double vy_fr_wheel = -vx_fr * sin_d + vy_fr * cos_d;

    // Calculate slip angles
    auto calcSlipAngle = [](double vx_w, double vy_w) -> double {
        if (std::abs(vx_w) < 0.5) {
            return 0.0;
        }
        return -std::atan2(vy_w, std::abs(vx_w));
    };

    state_.slip_angles[0] = calcSlipAngle(vx_fl_wheel, vy_fl_wheel);
    state_.slip_angles[1] = calcSlipAngle(vx_fr_wheel, vy_fr_wheel);
    state_.slip_angles[2] = calcSlipAngle(vx_rl, vy_rl);
    state_.slip_angles[3] = calcSlipAngle(vx_rr, vy_rr);

    // Calculate slip ratios (simplified - assume wheel speed matches vehicle speed)
    // In a more complete model, we'd track wheel angular velocities
    double wheel_radius = vehicle_params_.wheel_radius;
    for (int i = 0; i < 4; ++i) {
        state_.wheel_speeds[i] = std::abs(vx) / wheel_radius;
        state_.slip_ratios[i] = 0.0;  // Simplified: no longitudinal slip
    }

    // Add slip ratio from throttle/brake
    if (state_.input.throttle > 0.01) {
        // Rear wheel drive - add positive slip to rear wheels
        state_.slip_ratios[2] = state_.input.throttle * 0.1;
        state_.slip_ratios[3] = state_.input.throttle * 0.1;
    } else if (state_.input.brake > 0.01) {
        // All wheel braking - add negative slip
        for (int i = 0; i < 4; ++i) {
            state_.slip_ratios[i] = -state_.input.brake * 0.1;
        }
    }
}

void VehicleDynamics::calculateTireForces(double& Fx_total, double& Fy_total,
                                           double& Mz_total) {
    const double delta = state_.vehicle.steering;
    const double lf = vehicle_params_.cg_to_front;
    const double lr = vehicle_params_.cg_to_rear;
    const double tw = vehicle_params_.track_width / 2.0;

    Fx_total = 0.0;
    Fy_total = 0.0;
    Mz_total = 0.0;

    // Calculate forces for each wheel
    for (int i = 0; i < 4; ++i) {
        double Fx_wheel, Fy_wheel;
        tire_model_.calculateForces(state_.slip_ratios[i], state_.slip_angles[i],
                                    state_.normal_loads[i], Fx_wheel, Fy_wheel);
        tire_Fx_[i] = Fx_wheel;
        tire_Fy_[i] = Fy_wheel;
    }

    // Transform front wheel forces to vehicle frame
    double cos_d = std::cos(delta);
    double sin_d = std::sin(delta);

    double Fx_fl = tire_Fx_[0] * cos_d - tire_Fy_[0] * sin_d;
    double Fy_fl = tire_Fx_[0] * sin_d + tire_Fy_[0] * cos_d;
    double Fx_fr = tire_Fx_[1] * cos_d - tire_Fy_[1] * sin_d;
    double Fy_fr = tire_Fx_[1] * sin_d + tire_Fy_[1] * cos_d;

    // Rear wheel forces (no steering)
    double Fx_rl = tire_Fx_[2];
    double Fy_rl = tire_Fy_[2];
    double Fx_rr = tire_Fx_[3];
    double Fy_rr = tire_Fy_[3];

    // Sum forces
    Fx_total = Fx_fl + Fx_fr + Fx_rl + Fx_rr;
    Fy_total = Fy_fl + Fy_fr + Fy_rl + Fy_rr;

    // Calculate yaw moment
    Mz_total = lf * (Fy_fl + Fy_fr) - lr * (Fy_rl + Fy_rr) +
               tw * (Fx_fr - Fx_fl + Fx_rr - Fx_rl);

    // Add aerodynamic drag
    double Fx_aero, Fz_aero;
    calculateAeroForces(Fx_aero, Fz_aero);
    Fx_total += Fx_aero;
}

std::array<double, 6> VehicleDynamics::stateDerivative(const VehicleState& state,
                                                        const ControlInput& input) {
    // Temporarily set state for force calculations
    VehicleState saved_state = state_.vehicle;
    state_.vehicle = state;
    state_.input = input;

    calculateNormalLoads();
    calculateSlips();

    double Fx, Fy, Mz;
    calculateTireForces(Fx, Fy, Mz);

    // Restore state
    state_.vehicle = saved_state;

    const double m = vehicle_params_.mass;
    const double Iz = vehicle_params_.inertia_z;

    // State derivatives
    // dx/dt = vx * cos(yaw) - vy * sin(yaw)
    // dy/dt = vx * sin(yaw) + vy * cos(yaw)
    // dyaw/dt = yaw_rate
    // dvx/dt = (Fx / m) + vy * yaw_rate
    // dvy/dt = (Fy / m) - vx * yaw_rate
    // dyaw_rate/dt = Mz / Iz

    double cos_yaw = std::cos(state.yaw);
    double sin_yaw = std::sin(state.yaw);

    std::array<double, 6> deriv;
    deriv[0] = state.vx * cos_yaw - state.vy * sin_yaw;  // dx
    deriv[1] = state.vx * sin_yaw + state.vy * cos_yaw;  // dy
    deriv[2] = state.yaw_rate;                            // dyaw
    deriv[3] = (Fx / m) + state.vy * state.yaw_rate;     // dvx
    deriv[4] = (Fy / m) - state.vx * state.yaw_rate;     // dvy
    deriv[5] = Mz / Iz;                                   // dyaw_rate

    return deriv;
}

void VehicleDynamics::integrateRK4(const ControlInput& input, double dt) {
    VehicleState s0 = state_.vehicle;

    // k1
    auto k1 = stateDerivative(s0, input);

    // k2
    VehicleState s1 = s0;
    s1.x += k1[0] * dt / 2.0;
    s1.y += k1[1] * dt / 2.0;
    s1.yaw += k1[2] * dt / 2.0;
    s1.vx += k1[3] * dt / 2.0;
    s1.vy += k1[4] * dt / 2.0;
    s1.yaw_rate += k1[5] * dt / 2.0;
    auto k2 = stateDerivative(s1, input);

    // k3
    VehicleState s2 = s0;
    s2.x += k2[0] * dt / 2.0;
    s2.y += k2[1] * dt / 2.0;
    s2.yaw += k2[2] * dt / 2.0;
    s2.vx += k2[3] * dt / 2.0;
    s2.vy += k2[4] * dt / 2.0;
    s2.yaw_rate += k2[5] * dt / 2.0;
    auto k3 = stateDerivative(s2, input);

    // k4
    VehicleState s3 = s0;
    s3.x += k3[0] * dt;
    s3.y += k3[1] * dt;
    s3.yaw += k3[2] * dt;
    s3.vx += k3[3] * dt;
    s3.vy += k3[4] * dt;
    s3.yaw_rate += k3[5] * dt;
    auto k4 = stateDerivative(s3, input);

    // Update state (dynamic model result)
    VehicleState x_dyn = s0;
    x_dyn.x += (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]) * dt / 6.0;
    x_dyn.y += (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]) * dt / 6.0;
    x_dyn.yaw += (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]) * dt / 6.0;
    x_dyn.vx += (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]) * dt / 6.0;
    x_dyn.vy += (k1[4] + 2*k2[4] + 2*k3[4] + k4[4]) * dt / 6.0;
    x_dyn.yaw_rate += (k1[5] + 2*k2[5] + 2*k3[5] + k4[5]) * dt / 6.0;

    // Apply FSSIM-style kinematic correction for low-speed stability
    state_.vehicle = kinematicCorrection(x_dyn, s0, input, dt);

    // Normalize yaw to [-pi, pi]
    while (state_.vehicle.yaw > M_PI) state_.vehicle.yaw -= 2 * M_PI;
    while (state_.vehicle.yaw < -M_PI) state_.vehicle.yaw += 2 * M_PI;

    // Ensure non-negative forward velocity
    state_.vehicle.vx = std::max(0.0, state_.vehicle.vx);
}

VehicleState VehicleDynamics::kinematicCorrection(const VehicleState& x_dyn,
                                                   const VehicleState& x_prev,
                                                   const ControlInput& input,
                                                   double dt) {
    // FSSIM-style blending between kinematic and dynamic models
    // At low speeds, the dynamic model becomes unstable, so we blend in kinematic model
    // v < 1.5 m/s: pure kinematic
    // v > 2.0 m/s: pure dynamic
    // 1.5 < v < 2.0: linear blend

    VehicleState x_out = x_dyn;
    const double v = std::hypot(x_prev.vx, x_prev.vy);
    const double v_blend = 0.5 * (v - 1.5);  // 0 at v=1.5, 1 at v=3.5
    const double blend = std::clamp(v_blend, 0.0, 1.0);

    if (blend < 1.0) {
        // Kinematic model calculations
        const double L = vehicle_params_.wheelbase;
        const double lr = vehicle_params_.cg_to_rear;
        const double delta = x_prev.steering;

        // Kinematic bicycle model
        // v_y_kin = tan(delta) * v_x * l_r / L
        // r_kin = tan(delta) * v_x / L
        double v_y_kin = std::tan(delta) * x_dyn.vx * lr / L;
        double r_kin = std::tan(delta) * x_dyn.vx / L;

        // Blend dynamic and kinematic
        x_out.vy = blend * x_dyn.vy + (1.0 - blend) * v_y_kin;
        x_out.yaw_rate = blend * x_dyn.yaw_rate + (1.0 - blend) * r_kin;
    }

    return x_out;
}

void VehicleDynamics::applySteeringDynamics(double target, double dt) {
    // Clamp target to max steering
    target = std::clamp(target, -vehicle_params_.max_steering,
                        vehicle_params_.max_steering);

    // Rate limit
    double delta = target - state_.vehicle.steering;
    double max_delta = steering_rate_limit_ * dt;
    delta = std::clamp(delta, -max_delta, max_delta);

    state_.vehicle.steering += delta;
}

}  // namespace simulation_core
