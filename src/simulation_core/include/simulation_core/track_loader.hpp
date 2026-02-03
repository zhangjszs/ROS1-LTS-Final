#pragma once

#include "simulation_core/types.hpp"
#include <string>

namespace simulation_core {

/**
 * @brief Load track definitions from YAML files
 */
class TrackLoader {
public:
    TrackLoader() = default;

    /**
     * @brief Load track from YAML file
     * @param filepath Path to YAML file
     * @param track Output track definition
     * @return true if loading succeeded
     */
    bool load(const std::string& filepath, Track& track);

    /**
     * @brief Save track to YAML file
     * @param filepath Path to YAML file
     * @param track Track to save
     * @return true if saving succeeded
     */
    bool save(const std::string& filepath, const Track& track);

    /**
     * @brief Get last error message
     */
    const std::string& lastError() const { return last_error_; }

    /**
     * @brief Create a simple oval track for testing
     * @param length Track length [m]
     * @param width Track width [m]
     * @param cone_spacing Spacing between cones [m]
     * @return Generated track
     */
    static Track createOvalTrack(double length = 50.0, double width = 3.0,
                                  double cone_spacing = 5.0);

    /**
     * @brief Create a skidpad (figure-8) track
     * @param radius Circle radius [m]
     * @param cone_spacing Spacing between cones [m]
     * @return Generated track
     */
    static Track createSkidpadTrack(double radius = 9.125,
                                     double cone_spacing = 3.0);

    /**
     * @brief Create an acceleration track
     * @param length Track length [m]
     * @param width Track width [m]
     * @param cone_spacing Spacing between cones [m]
     * @return Generated track
     */
    static Track createAccelerationTrack(double length = 75.0, double width = 3.0,
                                          double cone_spacing = 5.0);

private:
    std::string last_error_;
};

}  // namespace simulation_core
