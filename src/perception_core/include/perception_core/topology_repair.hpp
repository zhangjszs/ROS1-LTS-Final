#pragma once

#include <vector>
#include <cmath>
#include <pcl/point_types.h>

namespace perception {

struct TopologyConfig {
    bool enable = false;
    double max_same_side_spacing = 5.0;   // Max gap before interpolation [m]
    double min_track_width = 2.5;         // Min expected track width [m]
    double max_track_width = 4.0;         // Max expected track width [m]
    double max_repair_range = 15.0;       // Only repair within this range [m]
    double outlier_lateral_threshold = 5.0; // Lateral distance for outlier rejection [m]
    double interpolated_confidence = 0.2; // Confidence for interpolated cones
};

struct TopologyCone {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double confidence = 0.0;
    bool is_interpolated = false;
    int original_index = -1;  // Index in original detection list, -1 if interpolated
};

class TopologyRepair {
public:
    TopologyRepair() = default;

    void setConfig(const TopologyConfig& config) { config_ = config; }

    /**
     * @brief Repair track topology: fill gaps and remove outliers
     * @param cones Input cone positions (x, y, z, confidence)
     * @return Repaired cone list (may include interpolated cones)
     */
    std::vector<TopologyCone> repair(const std::vector<TopologyCone>& cones);

private:
    /**
     * @brief Estimate track direction via PCA on cone centroids
     * @param cones Input cones
     * @param dir_x Output: track direction X component
     * @param dir_y Output: track direction Y component
     */
    void estimateTrackDirection(const std::vector<TopologyCone>& cones,
                                double& dir_x, double& dir_y);

    TopologyConfig config_;
};

}  // namespace perception
