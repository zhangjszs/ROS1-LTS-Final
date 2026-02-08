#include "perception_core/cone_tracker.hpp"
#include <algorithm>
#include <limits>
#include <numeric>

namespace perception {

void ConeTracker::update(const std::vector<Detection>& detections, double dt) {
    // 1. 预测步骤
    predict(dt);

    // 2. 数据关联 (Hungarian algorithm)
    auto associations = associate(detections);

    // 3. 更新已关联的轨迹
    std::vector<bool> det_used(detections.size(), false);
    for (const auto& [track_idx, det_idx] : associations) {
        updateTrack(tracks_[track_idx], detections[det_idx]);
        det_used[det_idx] = true;
    }

    // 4. 未关联的检测创建新轨迹
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!det_used[i]) {
            createTrack(detections[i]);
        }
    }

    // 5. 未关联的轨迹增加miss_count
    std::vector<bool> track_updated(tracks_.size(), false);
    for (const auto& [track_idx, det_idx] : associations) {
        track_updated[track_idx] = true;
    }
    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (!track_updated[i]) {
            tracks_[i].miss_count++;
            tracks_[i].hit_count = 0;
        }
    }

    // 6. 删除丢失的轨迹
    pruneDeadTracks();
}

void ConeTracker::predict(double /*dt*/) {
    // NOTE: In ego frame, static cones move backwards as the vehicle advances.
    // We do NOT predict positions here because we don't have odometry input.
    // Instead, we rely on a generous association_threshold to handle
    // frame-to-frame displacement caused by vehicle motion.
    // A future improvement would be to pass ego-motion (dx, dy, dyaw) from
    // odometry/IMU and transform all tracks before association.
}

std::vector<std::pair<int, int>> ConeTracker::associate(const std::vector<Detection>& detections) {
    if (tracks_.empty() || detections.empty()) {
        return {};
    }

    // Build cost matrix (Euclidean distance)
    const int n_tracks = static_cast<int>(tracks_.size());
    const int n_dets = static_cast<int>(detections.size());
    std::vector<std::vector<double>> cost_matrix(n_tracks,
        std::vector<double>(n_dets, std::numeric_limits<double>::max()));

    for (int i = 0; i < n_tracks; ++i) {
        for (int j = 0; j < n_dets; ++j) {
            double dx = tracks_[i].x - detections[j].x;
            double dy = tracks_[i].y - detections[j].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= config_.association_threshold) {
                cost_matrix[i][j] = dist;
            }
        }
    }

    return hungarianAssign(cost_matrix, n_tracks, n_dets, config_.association_threshold);
}

// PLACEHOLDER_HUNGARIAN

// Hungarian algorithm (Munkres) for optimal bipartite assignment
// Operates on a padded square cost matrix. Entries > threshold are treated as forbidden.
std::vector<std::pair<int, int>> ConeTracker::hungarianAssign(
    const std::vector<std::vector<double>>& cost_matrix,
    int n_rows, int n_cols, double threshold) {

    if (n_rows == 0 || n_cols == 0) return {};

    const double INF = threshold * 10.0;  // large sentinel
    const int n = std::max(n_rows, n_cols);

    // Pad to square matrix
    std::vector<std::vector<double>> C(n, std::vector<double>(n, INF));
    for (int i = 0; i < n_rows; ++i) {
        for (int j = 0; j < n_cols; ++j) {
            C[i][j] = (cost_matrix[i][j] <= threshold) ? cost_matrix[i][j] : INF;
        }
    }

    // Munkres algorithm (O(n^3))
    std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
    std::vector<int> p(n + 1, 0), way(n + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(n + 1, std::numeric_limits<double>::max());
        std::vector<bool> used(n + 1, false);

        do {
            used[j0] = true;
            int i0 = p[j0];
            double delta = std::numeric_limits<double>::max();
            int j1 = -1;

            for (int j = 1; j <= n; ++j) {
                if (!used[j]) {
                    double cur = C[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            if (j1 < 0) break;

            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }

            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    // Extract valid assignments
    std::vector<std::pair<int, int>> result;
    for (int j = 1; j <= n; ++j) {
        int i = p[j] - 1;
        int jj = j - 1;
        if (i >= 0 && i < n_rows && jj < n_cols &&
            cost_matrix[i][jj] <= threshold) {
            result.emplace_back(i, jj);
        }
    }
    return result;
}

void ConeTracker::updateTrack(TrackedCone& track, const Detection& det) {
    double alpha = config_.measurement_noise /
                   (config_.measurement_noise + config_.process_noise);

    // In ego frame, directly use measurement position (no velocity prediction)
    track.x = (1.0 - alpha) * track.x + alpha * det.x;
    track.y = (1.0 - alpha) * track.y + alpha * det.y;
    track.z = (1.0 - alpha) * track.z + alpha * det.z;

    track.confidence = 0.7 * track.confidence + 0.3 * det.confidence;
    track.color_type = det.color_type;

    track.hit_count++;
    track.miss_count = 0;

    if (track.hit_count >= config_.confirm_frames) {
        track.confirmed = true;
    }
}

void ConeTracker::createTrack(const Detection& det) {
    TrackedCone track;
    track.id = next_id_++;
    track.x = det.x;
    track.y = det.y;
    track.z = det.z;
    track.confidence = det.confidence;
    track.hit_count = 1;
    track.miss_count = 0;
    track.confirmed = false;
    track.vx = 0.0;
    track.vy = 0.0;
    track.color_type = det.color_type;

    tracks_.push_back(track);
}

void ConeTracker::pruneDeadTracks() {
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
            [this](const TrackedCone& track) {
                return track.miss_count >= config_.delete_frames;
            }),
        tracks_.end());
}

std::vector<ConeTracker::TrackedCone> ConeTracker::getConfirmedCones() const {
    std::vector<TrackedCone> confirmed;
    for (const auto& track : tracks_) {
        if (track.confirmed) {
            confirmed.push_back(track);
        }
    }
    return confirmed;
}

}  // namespace perception
