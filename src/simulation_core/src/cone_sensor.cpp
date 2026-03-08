#include "simulation_core/cone_sensor.hpp"
#include <cmath>
#include <algorithm>

namespace simulation_core {

ConeSensor::ConeSensor()
    : params_(),
      rng_(std::random_device{}()),
      noise_radial_(0.0, params_.lidar_noise_radial),
      noise_angular_(0.0, params_.lidar_noise_angular),
      uniform_(0.0, 1.0) {}

ConeSensor::ConeSensor(const SensorParams& params)
    : params_(params),
      rng_(std::random_device{}()),
      noise_radial_(0.0, params.lidar_noise_radial),
      noise_angular_(0.0, params.lidar_noise_angular),
      uniform_(0.0, 1.0) {}

void ConeSensor::setParams(const SensorParams& params) {
    params_ = params;
    noise_radial_ = std::normal_distribution<double>(0.0, params.lidar_noise_radial);
    noise_angular_ = std::normal_distribution<double>(0.0, params.lidar_noise_angular);
}

void ConeSensor::setTrack(const Track& track) {
    cones_ = track.cones;
}

void ConeSensor::setCones(const std::vector<Cone>& cones) {
    cones_ = cones;
}

void ConeSensor::observe(const VehicleState& state,
                          std::vector<ConeObservation>& observations) {
    observations.clear();

    for (const auto& cone : cones_) {
        ConeObservation obs;

        // Transform to vehicle frame and check FOV
        if (!transformToVehicleFrame(cone, state, obs)) {
            continue;
        }

        // Check range
        if (obs.distance > params_.lidar_max_range) {
            continue;
        }

        // Apply noise
        addNoise(obs);

        // Compute color probabilities
        computeColorProbabilities(cone.color, obs.distance, obs);

        // Set confidence based on distance
        double detection_prob = params_.lidar_detection_rate;
        if (obs.distance > params_.distance_dependent_detection) {
            detection_prob *= std::max(0.0, 1.0 - (obs.distance - params_.distance_dependent_detection) /
                                                   (params_.lidar_max_range - params_.distance_dependent_detection));
        }
        obs.confidence = detection_prob;

        observations.push_back(obs);
    }
}

void ConeSensor::addNoise(ConeObservation& obs) {
    // Add radial and angular noise
    double radial_noise = noise_radial_(rng_);
    double angular_noise = noise_angular_(rng_);

    double new_distance = std::max(0.0, obs.distance + radial_noise);
    double new_theta = obs.angle + angular_noise;

    obs.distance = new_distance;
    obs.angle = new_theta;
    obs.x = new_distance * std::cos(new_theta);
    obs.y = new_distance * std::sin(new_theta);
}

void ConeSensor::computeColorProbabilities(ConeColor true_color, double distance,
                                            ConeObservation& obs) {
    // Updated color probability computation
    // New cone types: BLUE=0, YELLOW_SMALL=1, YELLOW_BIG=2, RED=3, UNKNOWN=255
    // Note: ORANGE types removed, replaced with YELLOW_SMALL/YELLOW_BIG

    double color_accuracy = params_.camera_color_accuracy;

    // Distance-dependent color accuracy
    double misclass_factor = std::max(0.0, 1.0 - distance / params_.distance_dependent_misclass);
    color_accuracy *= misclass_factor;

    // Initialize probabilities (updated for new cone types)
    obs.prob_blue = 0.0f;
    obs.prob_yellow = 0.0f;
    obs.prob_orange = 0.0f;  // Kept for backward compatibility, always 0 now
    obs.prob_unknown = 0.0f;

    if (distance <= params_.camera_max_range) {
        // Within color detection range
        bool correct_color = uniform_(rng_) < color_accuracy;

        if (correct_color) {
            // Correct classification
            switch (true_color) {
                case ConeColor::BLUE:
                    obs.prob_blue = static_cast<float>(color_accuracy);
                    obs.prob_yellow = static_cast<float>(1.0 - color_accuracy);
                    obs.color = ConeColor::BLUE;
                    break;
                case ConeColor::YELLOW_SMALL:
                case ConeColor::YELLOW_BIG:
                    // Both yellow types classified as yellow
                    obs.prob_yellow = static_cast<float>(color_accuracy);
                    obs.prob_blue = static_cast<float>(1.0 - color_accuracy);
                    obs.color = ConeColor::YELLOW_SMALL;
                    break;
                case ConeColor::RED:
                    // Red cones - treat as a distinct type
                    obs.prob_blue = static_cast<float>((1.0 - color_accuracy) / 2.0);
                    obs.prob_yellow = static_cast<float>((1.0 - color_accuracy) / 2.0);
                    obs.color = ConeColor::RED;
                    break;
                default:
                    obs.prob_unknown = 1.0f;
                    obs.color = ConeColor::UNKNOWN;
            }
        } else {
            // Misclassification - swap blue/yellow
            if (true_color == ConeColor::BLUE) {
                obs.prob_yellow = static_cast<float>(color_accuracy);
                obs.prob_blue = static_cast<float>((1.0 - color_accuracy));
                obs.color = ConeColor::YELLOW_SMALL;
            } else if (true_color == ConeColor::YELLOW_SMALL || true_color == ConeColor::YELLOW_BIG) {
                obs.prob_blue = static_cast<float>(color_accuracy);
                obs.prob_yellow = static_cast<float>((1.0 - color_accuracy));
                obs.color = ConeColor::BLUE;
            } else if (true_color == ConeColor::RED) {
                // Red rarely misclassified, but may be confused with yellow
                obs.prob_yellow = static_cast<float>(color_accuracy * 0.5);
                obs.prob_blue = static_cast<float>(color_accuracy * 0.5);
                obs.color = ConeColor::RED;
            } else {
                obs.prob_unknown = 1.0f;
                obs.color = ConeColor::UNKNOWN;
            }
        }
    } else {
        // Beyond color detection range - uniform probability
        double d_after_color = distance - params_.camera_max_range;
        double d_range = params_.lidar_max_range - params_.camera_max_range;
        double lik_others = std::max(0.0, 1.0 - d_after_color / d_range);

        obs.prob_blue = static_cast<float>((1.0 - lik_others) / 2.0);
        obs.prob_yellow = static_cast<float>((1.0 - lik_others) / 2.0);
        obs.prob_orange = 0.0f;  // No orange cones anymore
        obs.prob_unknown = static_cast<float>(lik_others);
        obs.color = ConeColor::UNKNOWN;
    }
}

bool ConeSensor::colorMisclassify(ConeColor true_color, ConeColor& observed_color) {
    observed_color = true_color;

    // Only misclassify blue/yellow (not red)
    if (true_color != ConeColor::BLUE &&
        true_color != ConeColor::YELLOW_SMALL &&
        true_color != ConeColor::YELLOW_BIG) {
        return false;
    }

    // Check if misclassification occurs
    if (uniform_(rng_) > params_.camera_color_accuracy) {
        // Swap blue and yellow
        if (true_color == ConeColor::BLUE) {
            observed_color = ConeColor::YELLOW_SMALL;
        } else {
            observed_color = ConeColor::BLUE;
        }
        return true;
    }

    return false;
}

bool ConeSensor::transformToVehicleFrame(const Cone& cone, const VehicleState& state,
                                          ConeObservation& obs) const {
    // Transform from global to vehicle frame
    double dx = cone.x - state.x;
    double dy = cone.y - state.y;

    double cos_yaw = std::cos(-state.yaw);
    double sin_yaw = std::sin(-state.yaw);

    obs.x = dx * cos_yaw - dy * sin_yaw;
    obs.y = dx * sin_yaw + dy * cos_yaw;

    // Calculate distance and angle
    obs.distance = std::sqrt(obs.x * obs.x + obs.y * obs.y);
    obs.angle = std::atan2(obs.y, obs.x);

    // Check FOV
    if (std::abs(obs.angle) > params_.lidar_fov / 2.0) {
        return false;
    }

    // Set true color
    obs.color = cone.color;

    return true;
}

}  // namespace simulation_core
