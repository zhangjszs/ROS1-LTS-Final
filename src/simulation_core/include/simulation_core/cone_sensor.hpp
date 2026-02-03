#pragma once

#include "simulation_core/types.hpp"
#include <random>
#include <vector>

namespace simulation_core {

/**
 * @brief Simulated cone sensor (LiDAR + Camera fusion)
 *
 * Models cone detection with:
 * - Distance-dependent detection probability
 * - Position noise (radial and angular)
 * - Color misclassification
 * - Field of view limits
 */
class ConeSensor {
public:
    ConeSensor();
    explicit ConeSensor(const SensorParams& params);

    /**
     * @brief Set sensor parameters
     */
    void setParams(const SensorParams& params);

    /**
     * @brief Get current parameters
     */
    const SensorParams& params() const { return params_; }

    /**
     * @brief Set track cones for observation
     * @param track Track with cone positions
     */
    void setTrack(const Track& track);

    /**
     * @brief Set cones directly
     * @param cones Vector of cones
     */
    void setCones(const std::vector<Cone>& cones);

    /**
     * @brief Observe cones from current vehicle state
     * @param state Current vehicle state
     * @param observations Output detected cones (in vehicle frame)
     */
    void observe(const VehicleState& state,
                 std::vector<ConeObservation>& observations);

    /**
     * @brief Set random seed for reproducibility
     */
    void setSeed(unsigned int seed);

private:
    /**
     * @brief Calculate detection probability based on distance
     * @param distance Distance to cone [m]
     * @return Detection probability [0, 1]
     */
    double detectionProbability(double distance) const;

    /**
     * @brief Add noise to observation
     * @param obs Observation to modify
     */
    void addNoise(ConeObservation& obs);

    /**
     * @brief FSSIM-style color probability computation
     * Computes probability distribution over cone colors based on distance
     * @param true_color Actual cone color
     * @param distance Distance to cone [m]
     * @param obs Output observation with color probabilities
     */
    void computeColorProbabilities(ConeColor true_color, double distance,
                                    ConeObservation& obs);

    /**
     * @brief Potentially misclassify cone color
     * @param true_color Actual cone color
     * @param observed_color Output observed color
     * @return true if color was misclassified
     */
    bool colorMisclassify(ConeColor true_color, ConeColor& observed_color);

    /**
     * @brief Transform cone from global to vehicle frame
     * @param cone Global cone position
     * @param state Vehicle state
     * @param obs Output observation in vehicle frame
     * @return true if cone is in sensor FOV
     */
    bool transformToVehicleFrame(const Cone& cone, const VehicleState& state,
                                  ConeObservation& obs) const;

    SensorParams params_;
    std::vector<Cone> cones_;

    // Random number generation
    std::mt19937 rng_;
    std::normal_distribution<double> noise_radial_;
    std::normal_distribution<double> noise_angular_;
    std::uniform_real_distribution<double> uniform_;
};

}  // namespace simulation_core
