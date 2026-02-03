#include "perception_core/cone_tracker.hpp"
#include <algorithm>
#include <limits>

namespace perception {

void ConeTracker::update(const std::vector<Detection>& detections, double dt) {
    // 1. 预测步骤
    predict(dt);

    // 2. 数据关联
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

void ConeTracker::predict(double dt) {
    if (!config_.enable_velocity_prediction) {
        return;
    }

    for (auto& track : tracks_) {
        // 简单的匀速模型预测
        track.x += track.vx * dt;
        track.y += track.vy * dt;
    }
}

std::vector<std::pair<int, int>> ConeTracker::associate(const std::vector<Detection>& detections) {
    std::vector<std::pair<int, int>> associations;

    if (tracks_.empty() || detections.empty()) {
        return associations;
    }

    // 计算距离矩阵
    std::vector<std::vector<double>> dist_matrix(tracks_.size(),
        std::vector<double>(detections.size(), std::numeric_limits<double>::max()));

    for (size_t i = 0; i < tracks_.size(); ++i) {
        for (size_t j = 0; j < detections.size(); ++j) {
            double dx = tracks_[i].x - detections[j].x;
            double dy = tracks_[i].y - detections[j].y;
            dist_matrix[i][j] = std::sqrt(dx * dx + dy * dy);
        }
    }

    // 贪心关联（简化版匈牙利算法）
    std::vector<bool> track_matched(tracks_.size(), false);
    std::vector<bool> det_matched(detections.size(), false);

    while (true) {
        double min_dist = config_.association_threshold;
        int best_track = -1;
        int best_det = -1;

        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (track_matched[i]) continue;
            for (size_t j = 0; j < detections.size(); ++j) {
                if (det_matched[j]) continue;
                if (dist_matrix[i][j] < min_dist) {
                    min_dist = dist_matrix[i][j];
                    best_track = static_cast<int>(i);
                    best_det = static_cast<int>(j);
                }
            }
        }

        if (best_track < 0) break;

        associations.emplace_back(best_track, best_det);
        track_matched[best_track] = true;
        det_matched[best_det] = true;
    }

    return associations;
}

void ConeTracker::updateTrack(TrackedCone& track, const Detection& det) {
    // 卡尔曼滤波更新（简化版）
    double alpha = config_.measurement_noise /
                   (config_.measurement_noise + config_.process_noise);

    // 速度估计
    if (track.hit_count > 0) {
        track.vx = (1.0 - alpha) * track.vx + alpha * (det.x - track.x);
        track.vy = (1.0 - alpha) * track.vy + alpha * (det.y - track.y);
    }

    // 位置更新
    track.x = (1.0 - alpha) * track.x + alpha * det.x;
    track.y = (1.0 - alpha) * track.y + alpha * det.y;
    track.z = (1.0 - alpha) * track.z + alpha * det.z;

    // 置信度更新（指数移动平均）
    track.confidence = 0.7 * track.confidence + 0.3 * det.confidence;

    // 计数更新
    track.hit_count++;
    track.miss_count = 0;

    // 确认状态
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
