#pragma once

#include "perception_core/cluster_features.hpp"
#include "perception_core/cone_model_fitter.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace perception {

using PointType = pcl::PointXYZI;

class ConfidenceScorer {
public:
    struct Config {
        // Size constraints
        double min_height = 0.15;
        double max_height = 0.5;
        double min_area = 0.01;
        double max_area = 0.15;
        double max_box_altitude = 0.5;

        // Shape constraints
        double min_aspect_ratio = 1.5;
        double min_verticality = 0.8;

        // Density constraints
        double min_density_near = 50.0;
        double min_density_far = 10.0;
        double distance_threshold = 5.0;

        // Intensity constraints
        double min_intensity_mean = 30.0;

        // Weights
        double weight_size = 0.3;
        double weight_shape = 0.25;
        double weight_density = 0.2;
        double weight_intensity = 0.15;
        double weight_position = 0.1;

        // Road type (for backward compatibility)
        int road_type = 2;

        // Model fitting
        bool enable_model_fitting = true;
        double model_fit_bonus = 0.2;
        double model_fit_penalty = 0.15;

        // Track semantic constraints (neighbor-context scoring)
        struct TrackSemanticConfig {
            bool enable = false;
            double weight = 0.0;  // Weight of semantic dimension (recommended 0.15)
            double expected_track_width = 3.0;  // Expected track width [m]
            double expected_cone_spacing = 5.0; // Expected same-side spacing [m]
            double spacing_tolerance = 2.0;     // Tolerance for spacing score [m]
            double width_tolerance = 1.0;       // Tolerance for width score [m]
            double isolation_radius = 8.0;      // Radius to check for neighbors [m]
        };
        TrackSemanticConfig track_semantic;
    };

    ConfidenceScorer() = default;
    explicit ConfidenceScorer(const Config& config) : config_(config) {}

    void setConfig(const Config& config);
    double computeConfidence(const ClusterFeatures& features);
    double computeConfidenceWithFitting(const ClusterFeatures& features,
                                         const pcl::PointCloud<PointType>::Ptr& cluster);

    /**
     * @brief Compute confidence with neighbor context (track semantic scoring)
     * @param features Cluster features for this detection
     * @param cluster Point cloud of this cluster
     * @param all_centroids All cone centroids in current frame [x, y, z]
     * @param self_index Index of this detection in all_centroids
     * @return Confidence score [0.0, 1.0]
     */
    double computeConfidenceWithContext(const ClusterFeatures& features,
                                        const pcl::PointCloud<PointType>::Ptr& cluster,
                                        const std::vector<pcl::PointXYZ>& all_centroids,
                                        int self_index);

private:
    double scoreSizeConstraints(const ClusterFeatures& f);
    double scoreShapeConstraints(const ClusterFeatures& f);
    double scoreDensityConstraints(const ClusterFeatures& f);
    double scoreIntensityConstraints(const ClusterFeatures& f);
    double scorePositionConstraints(const ClusterFeatures& f);
    double scoreTrackSemanticConstraints(const pcl::PointXYZ& centroid,
                                          const std::vector<pcl::PointXYZ>& all_centroids,
                                          int self_index);

    Config config_;
    ConeModelFitter model_fitter_;
};

}  // namespace perception
