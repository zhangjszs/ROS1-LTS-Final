#pragma once

#include <Eigen/Core>
#include <vector>
#include <string>
#include <cstdint>
#include <cmath>

namespace simulation_core {

/**
 * @brief Vehicle state in simulation
 */
struct VehicleState {
    double x = 0.0;           // Global X position [m]
    double y = 0.0;           // Global Y position [m]
    double yaw = 0.0;         // Heading angle [rad]
    double vx = 0.0;          // Longitudinal velocity (body frame) [m/s]
    double vy = 0.0;          // Lateral velocity (body frame) [m/s]
    double yaw_rate = 0.0;    // Yaw rate [rad/s]
    double steering = 0.0;    // Current steering angle [rad]
};

/**
 * @brief Control input to vehicle
 */
struct ControlInput {
    double steering = 0.0;    // Steering angle command [rad]
    double throttle = 0.0;    // Throttle [0, 1]
    double brake = 0.0;       // Brake [0, 1]
};

/**
 * @brief Complete simulation state
 */
struct SimulationState {
    VehicleState vehicle;
    ControlInput input;
    double sim_time = 0.0;    // Simulation time [s]

    // Wheel states (FL, FR, RL, RR)
    double wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};
    double slip_ratios[4] = {0.0, 0.0, 0.0, 0.0};
    double slip_angles[4] = {0.0, 0.0, 0.0, 0.0};
    double normal_loads[4] = {0.0, 0.0, 0.0, 0.0};
};

/**
 * @brief Cone color enumeration
 */
enum class ConeColor : uint8_t {
    BLUE = 0,      // Right boundary
    YELLOW = 1,    // Left boundary
    ORANGE = 2,    // Start/finish
    ORANGE_BIG = 3,// Large orange cone
    UNKNOWN = 255
};

/**
 * @brief Cone position in track
 */
struct Cone {
    double x = 0.0;           // Global X [m]
    double y = 0.0;           // Global Y [m]
    ConeColor color = ConeColor::UNKNOWN;
};

/**
 * @brief Observed cone with noise and color probabilities
 * Inspired by FSSIM's probabilistic color model
 */
struct ConeObservation {
    double x = 0.0;           // Observed X (vehicle frame) [m]
    double y = 0.0;           // Observed Y (vehicle frame) [m]
    ConeColor color = ConeColor::UNKNOWN;
    double distance = 0.0;    // Distance from vehicle [m]
    double angle = 0.0;       // Angle from vehicle heading [rad]
    double confidence = 1.0;  // Detection confidence [0, 1]

    // FSSIM-style color probabilities
    float prob_blue = 0.0f;
    float prob_yellow = 0.0f;
    float prob_orange = 0.0f;
    float prob_unknown = 0.0f;
};

/**
 * @brief Track definition
 */
struct Track {
    std::string name;
    double start_x = 0.0;
    double start_y = 0.0;
    double start_heading = 0.0;
    std::vector<Cone> cones;
};

/**
 * @brief Vehicle parameters
 */
struct VehicleParams {
    // Mass and inertia
    double mass = 250.0;              // Total mass [kg]
    double inertia_z = 150.0;         // Yaw moment of inertia [kg*m^2]

    // Geometry
    double wheelbase = 1.55;          // Wheelbase [m]
    double track_width = 1.2;         // Track width [m]
    double cg_height = 0.3;           // CG height [m]
    double cg_to_front = 0.775;       // CG to front axle [m]
    double cg_to_rear = 0.775;        // CG to rear axle [m]

    // Wheel
    double wheel_radius = 0.23;       // Wheel radius [m]
    double wheel_inertia = 0.5;       // Wheel rotational inertia [kg*m^2]

    // Steering
    double max_steering = 0.5;        // Max steering angle [rad]
    double steering_ratio = 1.0;      // Steering ratio

    // Powertrain
    double max_torque = 200.0;        // Max motor torque [Nm]
    double max_brake_torque = 500.0;  // Max brake torque [Nm]

    // Aerodynamics
    double drag_coeff = 0.8;          // Drag coefficient
    double downforce_coeff = 2.5;     // Downforce coefficient
    double frontal_area = 1.0;        // Frontal area [m^2]
    double air_density = 1.225;       // Air density [kg/m^3]
};

/**
 * @brief Tire model parameters (Pacejka Magic Formula)
 */
struct TireParams {
    double B = 10.0;          // Stiffness factor
    double C = 1.65;          // Shape factor
    double D = 1.0;           // Peak factor
    double E = -0.5;          // Curvature factor
    double friction = 1.0;    // Friction coefficient
};

/**
 * @brief Sensor parameters
 */
struct SensorParams {
    // LiDAR
    double lidar_max_range = 20.0;        // Max detection range [m]
    double lidar_fov = M_PI;              // Field of view [rad]
    double lidar_noise_radial = 0.05;     // Radial noise std [m]
    double lidar_noise_angular = 0.007;   // Angular noise std [rad]
    double lidar_detection_rate = 0.95;   // Base detection probability

    // Camera (for color)
    double camera_max_range = 15.0;       // Max color detection range [m]
    double camera_color_accuracy = 0.9;   // Color classification accuracy
    double camera_delay = 0.1;            // Processing delay [s]

    // FSSIM-style distance-dependent parameters
    double distance_dependent_detection = 20.0;   // Distance at which detection prob = 0
    double distance_dependent_misclass = 15.0;    // Distance at which color accuracy = 0
    double delay_noise_sigma = 0.02;              // Delay noise std [s]
};

}  // namespace simulation_core
