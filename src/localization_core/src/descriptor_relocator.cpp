#include <localization_core/descriptor_relocator.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>

namespace localization_core {

DescriptorRelocator::DescriptorRelocator(const RelocConfig& cfg)
    : cfg_(cfg) {}

int DescriptorRelocator::descriptorSize() const {
  return cfg_.n_rings * cfg_.n_sectors * cfg_.n_channels;
}

std::vector<float> DescriptorRelocator::BuildDescriptor(
    const std::vector<FgLandmark>& landmarks,
    const Pose2& pose) const {
  const int desc_size = descriptorSize();
  std::vector<float> desc(desc_size, 0.0f);

  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);

  for (const auto& lm : landmarks) {
    // Transform to local frame
    const double dx = lm.x - pose.x;
    const double dy = lm.y - pose.y;
    const double lx = c * dx + s * dy;
    const double ly = -s * dx + c * dy;

    const double range = std::sqrt(lx * lx + ly * ly);
    if (range > cfg_.submap_radius || range < 0.5) continue;

    const double angle = std::atan2(ly, lx);  // [-pi, pi]

    // Quantize to ring and sector
    const int ring = std::min(
        static_cast<int>(range / cfg_.submap_radius * cfg_.n_rings),
        cfg_.n_rings - 1);
    int sector = static_cast<int>(
        (angle + M_PI) / (2.0 * M_PI) * cfg_.n_sectors);
    sector = std::clamp(sector, 0, cfg_.n_sectors - 1);

    // Color channel: 0=blue, 1=red (HUAT left boundary), 2=other/none
    int channel = 2;
    if (lm.color_type == 0) channel = 0;       // BLUE
    else if (lm.color_type == 5 || lm.color_type == 1) channel = 1;   // RED (fallback keeps legacy YELLOW)

    const int idx = (ring * cfg_.n_sectors + sector) * cfg_.n_channels + channel;
    if (idx >= 0 && idx < desc_size) {
      desc[idx] += 1.0f;
    }
  }

  // L2 normalize
  float norm = 0.0f;
  for (float v : desc) norm += v * v;
  norm = std::sqrt(norm);
  if (norm > 1e-6f) {
    for (float& v : desc) v /= norm;
  }

  return desc;
}

void DescriptorRelocator::AddToDatabase(const SubMapDescriptor& desc) {
  if (static_cast<int>(database_.size()) >= cfg_.max_descriptors) {
    // Evict oldest
    database_.erase(database_.begin());
  }
  database_.push_back(desc);
}

double DescriptorRelocator::cosineDistance(
    const std::vector<float>& a, const std::vector<float>& b) const {
  if (a.size() != b.size()) return 1.0;
  double dot = 0.0, na = 0.0, nb = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    dot += a[i] * b[i];
    na += a[i] * a[i];
    nb += b[i] * b[i];
  }
  const double denom = std::sqrt(na) * std::sqrt(nb);
  if (denom < 1e-9) return 1.0;
  return 1.0 - dot / denom;
}

RelocResult DescriptorRelocator::TryRelocalize(
    const std::vector<float>& current_desc,
    const std::vector<FgLandmark>& current_landmarks,
    const Pose2& current_pose) const {
  RelocResult result;
  if (database_.empty() || current_landmarks.size() < 3) return result;

  // Find top-K nearest descriptors by cosine distance
  struct Candidate {
    size_t db_idx;
    double dist;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(database_.size());
  for (size_t i = 0; i < database_.size(); ++i) {
    double d = cosineDistance(current_desc, database_[i].histogram);
    candidates.push_back({i, d});
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {
              return a.dist < b.dist;
            });

  const int k = std::min(cfg_.top_k, static_cast<int>(candidates.size()));
  const double inlier_thresh2 =
      cfg_.ransac_inlier_thresh * cfg_.ransac_inlier_thresh;

  // Convert current landmarks to local frame (relative to current_pose)
  struct Pt { double x; double y; };
  std::vector<Pt> cur_local(current_landmarks.size());
  {
    const double c = std::cos(current_pose.theta);
    const double s = std::sin(current_pose.theta);
    for (size_t i = 0; i < current_landmarks.size(); ++i) {
      const double dx = current_landmarks[i].x - current_pose.x;
      const double dy = current_landmarks[i].y - current_pose.y;
      cur_local[i] = {c * dx + s * dy, -s * dx + c * dy};
    }
  }

  std::mt19937 rng(42);

  for (int ci = 0; ci < k; ++ci) {
    const auto& db_desc = database_[candidates[ci].db_idx];
    if (db_desc.local_landmark_positions.size() < 2) continue;

    // Convert DB landmarks to local frame (relative to db_desc pose)
    const size_t n_db = db_desc.local_landmark_positions.size();
    std::vector<Pt> db_local(n_db);
    {
      const double c = std::cos(db_desc.ptheta);
      const double s = std::sin(db_desc.ptheta);
      for (size_t i = 0; i < n_db; ++i) {
        const double dx = db_desc.local_landmark_positions[i].first - db_desc.px;
        const double dy = db_desc.local_landmark_positions[i].second - db_desc.py;
        db_local[i] = {c * dx + s * dy, -s * dx + c * dy};
      }
    }

    // RANSAC: sample 2 point correspondences, estimate SE(2), count inliers
    // We match current local landmarks to DB local landmarks
    const size_t n_cur = cur_local.size();
    std::uniform_int_distribution<size_t> dist_cur(0, n_cur - 1);
    std::uniform_int_distribution<size_t> dist_db(0, n_db - 1);

    for (int iter = 0; iter < cfg_.ransac_max_iter; ++iter) {
      // Sample two distinct current landmark indices
      const size_t ci1 = dist_cur(rng);
      size_t ci2 = dist_cur(rng);
      if (ci2 == ci1) ci2 = (ci1 + 1) % n_cur;

      // For each sampled current landmark, find nearest DB landmark
      auto nearest_db = [&](const Pt& p) -> size_t {
        double best_d2 = 1e18;
        size_t best = 0;
        for (size_t j = 0; j < n_db; ++j) {
          const double ddx = p.x - db_local[j].x;
          const double ddy = p.y - db_local[j].y;
          const double d2 = ddx * ddx + ddy * ddy;
          if (d2 < best_d2) { best_d2 = d2; best = j; }
        }
        return best;
      };
      const size_t di1 = nearest_db(cur_local[ci1]);
      const size_t di2 = nearest_db(cur_local[ci2]);
      if (di1 == di2) continue;

      // Estimate SE(2) from 2 correspondences:
      // cur_local[ci] -> db_local[di]
      // T = [R t] such that db = R * cur + t
      const double ax = cur_local[ci1].x, ay = cur_local[ci1].y;
      const double bx = cur_local[ci2].x, by = cur_local[ci2].y;
      const double Ax = db_local[di1].x, Ay = db_local[di1].y;
      const double Bx = db_local[di2].x, By = db_local[di2].y;

      const double d_cur_x = bx - ax, d_cur_y = by - ay;
      const double d_db_x = Bx - Ax, d_db_y = By - Ay;
      const double len_cur = std::sqrt(d_cur_x * d_cur_x + d_cur_y * d_cur_y);
      const double len_db = std::sqrt(d_db_x * d_db_x + d_db_y * d_db_y);
      if (len_cur < 0.5 || len_db < 0.5) continue;
      // Reject if scale mismatch > 30%
      if (std::abs(len_cur - len_db) / std::max(len_cur, len_db) > 0.3) continue;

      const double theta_cur = std::atan2(d_cur_y, d_cur_x);
      const double theta_db = std::atan2(d_db_y, d_db_x);
      const double dtheta = std::atan2(std::sin(theta_db - theta_cur),
                                        std::cos(theta_db - theta_cur));
      const double ct = std::cos(dtheta), st = std::sin(dtheta);
      const double tx = Ax - (ct * ax - st * ay);
      const double ty = Ay - (st * ax + ct * ay);

      // Count inliers
      int inliers = 0;
      double total_res = 0.0;
      for (size_t i = 0; i < n_cur; ++i) {
        const double px = ct * cur_local[i].x - st * cur_local[i].y + tx;
        const double py = st * cur_local[i].x + ct * cur_local[i].y + ty;
        // Find nearest DB landmark
        double min_d2 = 1e18;
        for (size_t j = 0; j < n_db; ++j) {
          const double ddx = px - db_local[j].x;
          const double ddy = py - db_local[j].y;
          min_d2 = std::min(min_d2, ddx * ddx + ddy * ddy);
        }
        if (min_d2 < inlier_thresh2) {
          inliers++;
          total_res += std::sqrt(min_d2);
        }
      }

      if (inliers >= cfg_.min_inliers &&
          (inliers > result.inlier_count ||
           (inliers == result.inlier_count &&
            total_res / inliers < result.mean_residual))) {
        // Convert local-frame SE(2) to global correction:
        // The relative transform from current_pose to db_pose
        result.success = true;
        result.dx = db_desc.px - current_pose.x;
        result.dy = db_desc.py - current_pose.y;
        result.dtheta = std::atan2(
            std::sin(db_desc.ptheta - current_pose.theta + dtheta),
            std::cos(db_desc.ptheta - current_pose.theta + dtheta));
        result.inlier_count = inliers;
        result.mean_residual = total_res / inliers;
      }
    }
  }

  return result;
}

}  // namespace localization_core
