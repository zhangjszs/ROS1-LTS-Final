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

        // FSSIM-style distance-dependent detection probability
        double prob = detectionProbability(obs.distance);
        if (uniform_(rng_) > prob) {
            continue;
        }

        // Add noise (FSSIM-style radial/angular noise)
        addNoise(obs);

        // FSSIM-style color classification with probabilities
        computeColorProbabilities(cone.color, obs.distance, obs);

        // Set confidence based on distance
        obs.confidence = std::max(0.5, 1.0 - obs.distance / params_.lidar_max_range);

        observations.push_back(obs);
    }
}

void ConeSensor::setSeed(unsigned int seed) {
    rng_.seed(seed);
}

double ConeSensor::detectionProbability(double distance) const {
    // FSSIM-style: linear decrease with distance
    // P = base_rate * max(0, 1 - d / distance_dependent_detection)
    double factor = std::max(0.0, 1.0 - distance / params_.distance_dependent_detection);
    return params_.lidar_detection_rate * factor;
}

void ConeSensor::addNoise(ConeObservation& obs) {
    // FSSIM-style radial and angular noise
    double theta = std::atan2(obs.y, obs.x);
    double dr = noise_radial_(rng_);
    double dtheta = noise_angular_(rng_);

    // Apply radial noise
    double new_distance = std::max(0.1, obs.distance + dr);

    // Apply angular noise
    double new_theta = theta + dtheta;

    // Update observation
    obs.distance = new_distance;
    obs.angle = new_theta;
    obs.x = new_distance * std::cos(new_theta);
    obs.y = new_distance * std::sin(new_theta);
}

void ConeSensor::computeColorProbabilities(ConeColor true_color, double distance,
                                            ConeObservation& obs) {
    // FSSIM-style color probability computation
    // Within color_observation_radius: high accuracy
    // Beyond: accuracy decreases linearly

    double color_accuracy = params_.camera_color_accuracy;

    // Distance-dependent color accuracy (FSSIM style)
    double misclass_factor = std::max(0.0, 1.0 - distance / params_.distance_dependent_misclass);
    color_accuracy *= misclass_factor;

    // Initialize probabilities
    obs.prob_blue = 0.0f;
    obs.prob_yellow = 0.0f;
    obs.prob_orange = 0.0f;
    obs.prob_unknown = 0.0f;

    if (distance <= params_.camera_max_range) {
        // Within color detection range
        bool correct_color = uniform_(rng_) < color_accuracy;

        if (correct_color) {
            // Correct classification
            switch (true_color) {
                case ConeColor::BLUE:
                    obs.prob_blue = static_cast<float>(color_accuracy);
                    obs.prob_yellow = static_cast<float>((1.0 - color_accuracy) / 2.0);
                    obs.prob_orange = static_cast<float>((1.0 - color_accuracy) / 2.0);
                    obs.color = ConeColor::BLUE;
                    break;
                case ConeColor::YELLOW:
                    obs.prob_yellow = static_cast<float>(color_accuracy);
                    obs.prob_blue = static_cast<float>((1.0 - color_accuracy) / 2.0);
                    obs.prob_orange = static_cast<float>((1.0 - color_accuracy) / 2.0);
                    obs.color = ConeColor::YELLOW;
                    break;
                case ConeColor::ORANGE:
                case ConeColor::ORANGE_BIG:
                    obs.prob_orange = static_cast<float>(color_accuracy);
                    obs.prob_blue = static_cast<float>((1.0 - color_accuracy) / 2.0);
                    obs.prob_yellow = static_cast<float>((1.0 - color_accuracy) / 2.0);
                    obs.color = ConeColor::ORANGE;
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
                obs.color = ConeColor::YELLOW;
            } else if (true_color == ConeColor::YELLOW) {
                obs.prob_blue = static_cast<float>(color_accuracy);
                obs.prob_yellow = static_cast<float>((1.0 - color_accuracy));
                obs.color = ConeColor::BLUE;
            } else {
                // Orange cones rarely misclassified
                obs.prob_orange = static_cast<float>(color_accuracy);
                obs.color = ConeColor::ORANGE;
            }
        }
    } else {
        // Beyond color detection range - uniform probability
        // FSSIM style: probability decreases linearly to unknown
        double d_after_color = distance - params_.camera_max_range;
        double d_range = params_.lidar_max_range - params_.camera_max_range;
        double lik_others = std::max(0.0, 1.0 - d_after_color / d_range);

        obs.prob_blue = static_cast<float>((1.0 - lik_others) / 3.0);
        obs.prob_yellow = static_cast<float>((1.0 - lik_others) / 3.0);
        obs.prob_orange = static_cast<float>((1.0 - lik_others) / 3.0);
        obs.prob_unknown = static_cast<float>(lik_others);
        obs.color = ConeColor::UNKNOWN;
    }
}

bool ConeSensor::colorMisclassify(ConeColor true_color, ConeColor& observed_color) {
    observed_color = true_color;

    // Only misclassify blue/yellow
    if (true_color != ConeColor::BLUE && true_color != ConeColor::YELLOW) {
        return false;
    }

    // Check if misclassification occurs
    if (uniform_(rng_) > params_.camera_color_accuracy) {
        // Swap blue and yellow
        if (true_color == ConeColor::BLUE) {
            observed_color = ConeColor::YELLOW;
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
    obs.distance = std::hypot(obs.x, obs.y);
    obs.angle = std::atan2(obs.y, obs.x);

    // Check if within FOV (front half-plane + FOV angle)
    double half_fov = params_.lidar_fov / 2.0;
    if (std::abs(obs.angle) > half_fov) {
        return false;
    }

    // Must be in front of vehicle
    if (obs.x < 0) {
        return false;
    }

    return true;
}

}  // namespace simulation_core
