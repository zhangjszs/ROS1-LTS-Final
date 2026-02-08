#include "perception_core/topology_repair.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <limits>

namespace perception {

void TopologyRepair::estimateTrackDirection(
    const std::vector<TopologyCone>& cones,
    double& dir_x, double& dir_y) {

    if (cones.size() < 2) {
        dir_x = 1.0;
        dir_y = 0.0;
        return;
    }

    // Compute centroid
    double cx = 0.0, cy = 0.0;
    for (const auto& c : cones) {
        cx += c.x;
        cy += c.y;
    }
    cx /= static_cast<double>(cones.size());
    cy /= static_cast<double>(cones.size());

    // PCA: compute covariance matrix [cxx cxy; cxy cyy]
    double cxx = 0.0, cxy = 0.0, cyy = 0.0;
    for (const auto& c : cones) {
        double dx = c.x - cx;
        double dy = c.y - cy;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }

    // First eigenvector of 2x2 symmetric matrix (largest eigenvalue)
    double trace = cxx + cyy;
    double det = cxx * cyy - cxy * cxy;
    double disc = std::sqrt(std::max(0.0, trace * trace / 4.0 - det));
    // lambda1 = trace/2 + disc (largest eigenvalue)
    double lambda1 = trace / 2.0 + disc;

    // Eigenvector for lambda1: (cxy, lambda1 - cxx) or (lambda1 - cyy, cxy)
    if (std::abs(cxy) > 1e-9) {
        dir_x = cxy;
        dir_y = lambda1 - cxx;
    } else if (cxx >= cyy) {
        dir_x = 1.0;
        dir_y = 0.0;
    } else {
        dir_x = 0.0;
        dir_y = 1.0;
    }

    // Normalize
    double len = std::sqrt(dir_x * dir_x + dir_y * dir_y);
    if (len > 1e-9) {
        dir_x /= len;
        dir_y /= len;
    } else {
        dir_x = 1.0;
        dir_y = 0.0;
    }

    // Ensure direction points forward (positive x in base_link)
    if (dir_x < 0.0) {
        dir_x = -dir_x;
        dir_y = -dir_y;
    }
}

std::vector<TopologyCone> TopologyRepair::repair(
    const std::vector<TopologyCone>& cones) {

    if (!config_.enable || cones.size() < 3) {
        return cones;
    }

    // Filter to cones within repair range
    std::vector<TopologyCone> local;
    std::vector<TopologyCone> far_cones;
    for (const auto& c : cones) {
        double dist = std::sqrt(c.x * c.x + c.y * c.y);
        if (dist <= config_.max_repair_range) {
            local.push_back(c);
        } else {
            far_cones.push_back(c);
        }
    }

    if (local.size() < 3) {
        return cones;
    }

    // Step 1: Estimate track direction via PCA
    double dir_x, dir_y;
    estimateTrackDirection(local, dir_x, dir_y);

    // Lateral axis (perpendicular to track direction)
    double lat_x = -dir_y;
    double lat_y = dir_x;

    // Step 2: Project cones onto track and lateral axes
    struct ProjectedCone {
        double along;   // projection onto track direction
        double lateral;  // projection onto lateral axis
        size_t idx;
    };
    std::vector<ProjectedCone> projected;
    projected.reserve(local.size());
    for (size_t i = 0; i < local.size(); ++i) {
        ProjectedCone pc;
        pc.along = local[i].x * dir_x + local[i].y * dir_y;
        pc.lateral = local[i].x * lat_x + local[i].y * lat_y;
        pc.idx = i;
        projected.push_back(pc);
    }

    // Step 3: Split into left/right by sign of lateral projection
    std::vector<size_t> left_idx, right_idx;
    for (const auto& pc : projected) {
        if (pc.lateral >= 0.0) {
            left_idx.push_back(pc.idx);
        } else {
            right_idx.push_back(pc.idx);
        }
    }

    // Step 4: Sort each side by along-track projection
    auto sort_by_along = [&](std::vector<size_t>& indices) {
        std::sort(indices.begin(), indices.end(),
            [&](size_t a, size_t b) {
                double along_a = local[a].x * dir_x + local[a].y * dir_y;
                double along_b = local[b].x * dir_x + local[b].y * dir_y;
                return along_a < along_b;
            });
    };
    sort_by_along(left_idx);
    sort_by_along(right_idx);

    // Step 5: Remove outliers (lateral projection too far from track)
    std::vector<bool> is_outlier(local.size(), false);
    for (size_t i = 0; i < local.size(); ++i) {
        double lat = std::abs(local[i].x * lat_x + local[i].y * lat_y);
        if (lat > config_.outlier_lateral_threshold) {
            is_outlier[i] = true;
        }
    }

    // Step 6: Fill gaps on each side
    // PLACEHOLDER_FILL_GAPS

    // Build result: non-outlier local cones + far cones + interpolated
    std::vector<TopologyCone> result;
    result.reserve(cones.size() + 10);

    for (size_t i = 0; i < local.size(); ++i) {
        if (!is_outlier[i]) {
            result.push_back(local[i]);
        }
    }

    // Fill gaps helper
    auto fillGaps = [&](const std::vector<size_t>& sorted_indices) {
        for (size_t k = 1; k < sorted_indices.size(); ++k) {
            size_t i0 = sorted_indices[k - 1];
            size_t i1 = sorted_indices[k];
            if (is_outlier[i0] || is_outlier[i1]) continue;

            double dx = local[i1].x - local[i0].x;
            double dy = local[i1].y - local[i0].y;
            double spacing = std::sqrt(dx * dx + dy * dy);

            if (spacing > config_.max_same_side_spacing) {
                // Insert interpolated cone at midpoint
                TopologyCone interp;
                interp.x = (local[i0].x + local[i1].x) / 2.0;
                interp.y = (local[i0].y + local[i1].y) / 2.0;
                interp.z = (local[i0].z + local[i1].z) / 2.0;
                interp.confidence = config_.interpolated_confidence;
                interp.is_interpolated = true;
                interp.original_index = -1;
                result.push_back(interp);
            }
        }
    };

    fillGaps(left_idx);
    fillGaps(right_idx);

    // Add far cones unchanged
    result.insert(result.end(), far_cones.begin(), far_cones.end());

    return result;
}

}  // namespace perception
