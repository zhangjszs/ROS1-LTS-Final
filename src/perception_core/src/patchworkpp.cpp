#include "perception_core/patchworkpp.hpp"

#include <algorithm>
#include <limits>

namespace patchwork {

namespace {

bool point_z_cmp(const PointXYZ &a, const PointXYZ &b) { return a.z < b.z; }

}  // namespace

Params::Params()
    : verbose(false),
      enable_RNR(true),
      enable_RVPF(true),
      enable_TGR(true),
      num_iter(3),
      num_lpr(20),
      num_min_pts(10),
      num_zones(4),
      num_rings_of_interest(4),
      RNR_ver_angle_thr(-15.0),
      RNR_intensity_thr(0.2),
      sensor_height(1.723),
      th_seeds(0.125),
      th_dist(0.125),
      th_seeds_v(0.25),
      th_dist_v(0.1),
      max_range(80.0),
      min_range(2.7),
      uprightness_thr(0.707),
      adaptive_seed_selection_margin(-1.2),
      intensity_thr(0.0),
      num_sectors_each_zone({16, 32, 54, 32}),
      num_rings_each_zone({2, 4, 4, 4}),
      max_flatness_storage(1000),
      max_elevation_storage(1000),
      elevation_thr({0, 0, 0, 0}),
      flatness_thr({0, 0, 0, 0}),
      th_dist_far_scale(1.5),
      min_normal_z(0.7),
      far_zone_min_pts_scale(2.0)
{
}

PatchWorkpp::PatchWorkpp(Params params) : params_(std::move(params))
{
  const double min_range_z2_ = (7 * params_.min_range + params_.max_range) / 8.0;
  const double min_range_z3_ = (3 * params_.min_range + params_.max_range) / 4.0;
  const double min_range_z4_ = (params_.min_range + params_.max_range) / 2.0;
  min_ranges_ = {params_.min_range, min_range_z2_, min_range_z3_, min_range_z4_};

  ring_sizes_ = {(min_range_z2_ - params_.min_range) / params_.num_rings_each_zone.at(0),
                 (min_range_z3_ - min_range_z2_) / params_.num_rings_each_zone.at(1),
                 (min_range_z4_ - min_range_z3_) / params_.num_rings_each_zone.at(2),
                 (params_.max_range - min_range_z4_) / params_.num_rings_each_zone.at(3)};
  sector_sizes_ = {2 * M_PI / params_.num_sectors_each_zone.at(0),
                   2 * M_PI / params_.num_sectors_each_zone.at(1),
                   2 * M_PI / params_.num_sectors_each_zone.at(2),
                   2 * M_PI / params_.num_sectors_each_zone.at(3)};

  for (int k = 0; k < params_.num_zones; k++) {
    Ring empty_ring;
    empty_ring.resize(params_.num_sectors_each_zone[k]);

    Zone z;
    for (int i = 0; i < params_.num_rings_each_zone[k]; i++) {
      z.push_back(empty_ring);
    }

    ConcentricZoneModel_.push_back(z);
  }

  std::cout << "PatchWorkpp::PatchWorkpp() - INITIALIZATION COMPLETE" << std::endl;
}

Eigen::MatrixX3f PatchWorkpp::toEigenCloud(const std::vector<PointXYZ> &cloud) const
{
  Eigen::MatrixX3f dst(static_cast<int>(cloud.size()), 3);
  int j = 0;
  for (const auto &p : cloud) {
    dst.row(j++) << p.x, p.y, p.z;
  }
  return dst;
}

Eigen::VectorXi PatchWorkpp::toIndices(const std::vector<PointXYZ> &cloud) const
{
  Eigen::VectorXi dst(static_cast<int>(cloud.size()));
  int j = 0;
  for (const auto &p : cloud) {
    dst.row(j++) << p.idx;
  }
  return dst;
}

void PatchWorkpp::addCloud(std::vector<PointXYZ> &cloud, const std::vector<PointXYZ> &add)
{
  cloud.insert(cloud.end(), add.begin(), add.end());
}

void PatchWorkpp::flush_patches(std::vector<Zone> &czm)
{
  for (int k = 0; k < params_.num_zones; k++) {
    for (int i = 0; i < params_.num_rings_each_zone[k]; i++) {
      for (int j = 0; j < params_.num_sectors_each_zone[k]; j++) {
        czm[k][i][j].clear();
      }
    }
  }

  if (params_.verbose) {
    std::cout << "\033[1;31m"
              << "PatchWorkpp::flush_patches() - Flushed patches successfully!"
              << "\033[0m" << std::endl;
  }
}

void PatchWorkpp::estimate_plane(const std::vector<PointXYZ> &ground)
{
  if (ground.empty()) {
    return;
  }

  Eigen::MatrixX3f eigen_ground(static_cast<int>(ground.size()), 3);
  int j = 0;
  for (const auto &p : ground) {
    eigen_ground.row(j++) << p.x, p.y, p.z;
  }
  const Eigen::MatrixX3f centered = eigen_ground.rowwise() - eigen_ground.colwise().mean();
  const Eigen::MatrixX3f cov =
      (centered.adjoint() * centered) / static_cast<double>(eigen_ground.rows() - 1);

  pc_mean_.resize(3);
  pc_mean_ << eigen_ground.colwise().mean()(0), eigen_ground.colwise().mean()(1),
      eigen_ground.colwise().mean()(2);

  Eigen::JacobiSVD<Eigen::MatrixX3f> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  singular_values_ = svd.singularValues();

  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));

  if (normal_(2) < 0) {
    for (int i = 0; i < 3; i++) {
      normal_(i) *= -1;
    }
  }

  // 法向量约束：拒绝过于水平的平面
  if (normal_(2) < params_.min_normal_z) {
    // 法向量Z分量过小，使用默认垂直地面
    normal_ << 0, 0, 1;
  }

  // mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose() * seeds_mean)(0, 0);
}

void PatchWorkpp::extract_initial_seeds(const int zone_idx,
                                        const std::vector<PointXYZ> &p_sorted,
                                        std::vector<PointXYZ> &init_seeds,
                                        double th_seed)
{
  init_seeds.clear();

  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;

  int init_idx = 0;
  if (zone_idx == 0) {
    for (size_t i = 0; i < p_sorted.size(); i++) {
      if (p_sorted[i].z < params_.adaptive_seed_selection_margin * params_.sensor_height) {
        ++init_idx;
      } else {
        break;
      }
    }
  }

  // Calculate the mean height value.
  for (size_t i = static_cast<size_t>(init_idx); i < p_sorted.size() && cnt < params_.num_lpr;
       i++) {
    sum += p_sorted[i].z;
    cnt++;
  }
  const double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0

  // iterate pointcloud, filter those height is less than lpr.height+params_.th_seeds
  for (size_t i = 0; i < p_sorted.size(); i++) {
    if (p_sorted[i].z < lpr_height + th_seed) {
      init_seeds.push_back(p_sorted[i]);
    }
  }
}

void PatchWorkpp::extract_initial_seeds(const int zone_idx,
                                        const std::vector<PointXYZ> &p_sorted,
                                        std::vector<PointXYZ> &init_seeds)
{
  init_seeds.clear();

  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;

  int init_idx = 0;
  if (zone_idx == 0) {
    for (size_t i = 0; i < p_sorted.size(); i++) {
      if (p_sorted[i].z < params_.adaptive_seed_selection_margin * params_.sensor_height) {
        ++init_idx;
      } else {
        break;
      }
    }
  }

  // Calculate the mean height value.
  for (size_t i = static_cast<size_t>(init_idx); i < p_sorted.size() && cnt < params_.num_lpr;
       i++) {
    sum += p_sorted[i].z;
    cnt++;
  }
  const double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0

  // iterate pointcloud, filter those height is less than lpr.height+params_.th_seeds
  for (size_t i = 0; i < p_sorted.size(); i++) {
    if (p_sorted[i].z < lpr_height + params_.th_seeds) {
      init_seeds.push_back(p_sorted[i]);
    }
  }
}

void PatchWorkpp::estimateGround(Eigen::MatrixXf cloud_in)
{
  cloud_ground_.clear();
  cloud_nonground_.clear();

  if (params_.verbose) {
    std::cout << "\033[1;32m"
              << "PatchWorkpp::estimateGround() - Estimation starts !"
              << "\033[0m" << std::endl;
  }

  clock_t beg = clock();

  // 1. Reflected Noise Removal (RNR)
  if (params_.enable_RNR) {
    reflected_noise_removal(cloud_in);
  }

  clock_t t1 = clock();

  // 2. Concentric Zone Model (CZM)
  flush_patches(ConcentricZoneModel_);

  clock_t t1_1 = clock();

  pc2czm(cloud_in, ConcentricZoneModel_);

  clock_t t2 = clock();

  int concentric_idx = 0;

  centers_.clear();
  normals_.clear();

  double t_sort = 0.0;
  double t_pca = 0.0;
  double t_gle = 0.0;
  double t_revert = 0.0;
  double t_update = 0.0;

  std::vector<patchwork::RevertCandidate> candidates;
  std::vector<double> ringwise_flatness;

  for (int zone_idx = 0; zone_idx < params_.num_zones; ++zone_idx) {
    const auto &zone = ConcentricZoneModel_[zone_idx];

    for (int ring_idx = 0; ring_idx < params_.num_rings_each_zone[zone_idx]; ++ring_idx) {
      for (int sector_idx = 0; sector_idx < params_.num_sectors_each_zone[zone_idx];
           ++sector_idx) {
        if (zone[ring_idx][sector_idx].size() < static_cast<size_t>(params_.num_min_pts)) {
          addCloud(cloud_nonground_, zone[ring_idx][sector_idx]);
          continue;
        }

        // --------- region-wise sorting (faster than global sorting method) ---------------- //
        clock_t t_bef_sort = clock();
        std::vector<PointXYZ> sorted = zone[ring_idx][sector_idx];
        std::sort(sorted.begin(), sorted.end(), point_z_cmp);
        clock_t t_aft_sort = clock();

        t_sort += t_aft_sort - t_bef_sort;
        // ---------------------------------------------------------------------------------- //

        clock_t t_bef_pca = clock();
        extract_piecewiseground(zone_idx, sorted, regionwise_ground_, regionwise_nonground_);
        clock_t t_aft_pca = clock();

        t_pca += t_aft_pca - t_bef_pca;

        centers_.push_back(PointXYZ(pc_mean_(0), pc_mean_(1), pc_mean_(2)));
        normals_.push_back(PointXYZ(normal_(0), normal_(1), normal_(2)));

        clock_t t_bef_gle = clock();
        // Status of each patch
        // used in checking uprightness, elevation, and flatness, respectively
        const double ground_uprightness = normal_(2);
        const double ground_elevation = pc_mean_(2);
        const double ground_flatness = singular_values_.minCoeff();
        const double line_variable =
            singular_values_(1) != 0 ? singular_values_(0) / singular_values_(1)
                                     : std::numeric_limits<double>::max();

        double heading = 0.0;
        for (int i = 0; i < 3; i++) {
          heading += pc_mean_(i) * normal_(i);
        }

        /*
            About 'is_heading_outside' condition, heading should be smaller than 0 theoretically.
            ( Imagine the geometric relationship between the surface normal vector on the ground
           plane and the vector connecting the sensor origin and the mean point of the ground plane
           )

            However, when the patch is far away from the sensor origin,
            heading could be larger than 0 even if it's ground due to lack of amount of ground plane
           points.

            Therefore, we only check this value when concentric_idx < num_rings_of_interest ( near
           condition )
        */
        const bool is_upright = ground_uprightness > params_.uprightness_thr;
        const bool is_near_zone = concentric_idx < params_.num_rings_of_interest;
        const bool is_heading_outside = heading < 0.0;

        bool is_not_elevated = false;
        bool is_flat = false;

        if (concentric_idx < params_.num_rings_of_interest) {
          is_not_elevated = ground_elevation < params_.elevation_thr[concentric_idx];
          is_flat = ground_flatness < params_.flatness_thr[concentric_idx];
        }

        /*
            Store the elevation & flatness variables
            for A-GLE (Adaptive Ground Likelihood Estimation)
            and TGR (Temporal Ground Revert). More information in the paper Patchwork++.
        */
        if (is_upright && is_not_elevated && is_near_zone) {
          update_elevation_[concentric_idx].push_back(ground_elevation);
          update_flatness_[concentric_idx].push_back(ground_flatness);

          ringwise_flatness.push_back(ground_flatness);
        }

        // Ground estimation based on conditions
        if (!is_upright) {
          addCloud(cloud_nonground_, regionwise_ground_);
        } else if (!is_near_zone) {
          // 远区优化：结合法向量垂直性、点密度和平面拟合质量
          // line_variable 表示平面拟合的线性程度，值越大说明点云越接近线状（非平面）
          const int far_min_pts = static_cast<int>(params_.num_min_pts * params_.far_zone_min_pts_scale);
          const bool enough_points = regionwise_ground_.size() >= static_cast<size_t>(far_min_pts);
          const bool is_good_plane = line_variable < 0.2;  // 平面拟合质量好
          const bool is_vertical_enough = ground_uprightness > 0.6;  // 法向量足够垂直
          
          if (is_good_plane && is_vertical_enough && enough_points) {
            addCloud(cloud_ground_, regionwise_ground_);
          } else {
            addCloud(cloud_nonground_, regionwise_ground_);
          }
        } else if (!is_heading_outside) {
          addCloud(cloud_nonground_, regionwise_ground_);
        } else if (is_not_elevated || is_flat) {
          addCloud(cloud_ground_, regionwise_ground_);
        } else {
          patchwork::RevertCandidate candidate(concentric_idx,
                                               sector_idx,
                                               ground_flatness,
                                               line_variable,
                                               pc_mean_,
                                               regionwise_ground_);
          candidates.push_back(candidate);
        }
        // Every regionwise_nonground is considered nonground.
        addCloud(cloud_nonground_, regionwise_nonground_);

        clock_t t_aft_gle = clock();

        t_gle += t_aft_gle - t_bef_gle;
      }

      clock_t t_bef_revert = clock();
      if (!candidates.empty()) {
        if (params_.enable_TGR) {
          temporal_ground_revert(ringwise_flatness, candidates, concentric_idx);
        } else {
          for (const auto &candidate : candidates) {
            addCloud(cloud_nonground_, candidate.regionwise_ground);
          }
        }

        candidates.clear();
        ringwise_flatness.clear();
      }
      clock_t t_aft_revert = clock();

      t_revert += t_aft_revert - t_bef_revert;

      concentric_idx++;
    }
  }

  clock_t t_bef_update = clock();
  update_elevation_thr();
  update_flatness_thr();
  clock_t t_aft_update = clock();

  t_update = t_aft_update - t_bef_update;

  clock_t end = clock();
  time_taken_ = end - beg;

  if (params_.verbose) {
    std::cout << "Time taken : " << time_taken_ / static_cast<double>(1000000) << "(sec) ~ "
              << (t2 - t1_1) / static_cast<double>(1000000) << "(czm) + "
              << t_sort / static_cast<double>(1000000) << "(sort) + "
              << t_pca / static_cast<double>(1000000) << "(pca) + "
              << t_gle / static_cast<double>(1000000) << "(estimate)" << std::endl;
  }

  if (params_.verbose) {
    std::cout << "\033[1;32m"
              << "PatchWorkpp::estimateGround() - Estimation is finished !"
              << "\033[0m" << std::endl;
  }
}

void PatchWorkpp::update_elevation_thr(void)
{
  for (int i = 0; i < params_.num_rings_of_interest; i++) {
    if (update_elevation_[i].empty()) {
      continue;
    }

    double update_mean = 0.0;
    double update_stdev = 0.0;
    calc_mean_stdev(update_elevation_[i], update_mean, update_stdev);
    if (i == 0) {
      params_.elevation_thr[i] = update_mean + 3 * update_stdev;
      // 注意：动态更新sensor_height可能导致不稳定，建议使用配置值
      // params_.sensor_height = -update_mean;
    } else {
      params_.elevation_thr[i] = update_mean + 2 * update_stdev;
    }

    int exceed_num = static_cast<int>(update_elevation_[i].size()) - params_.max_elevation_storage;
    if (exceed_num > 0) {
      update_elevation_[i].erase(update_elevation_[i].begin(),
                                 update_elevation_[i].begin() + exceed_num);
    }
  }
}

void PatchWorkpp::update_flatness_thr(void)
{
  for (int i = 0; i < params_.num_rings_of_interest; i++) {
    if (update_flatness_[i].empty()) {
      break;
    }
    if (update_flatness_[i].size() <= 1) {
      break;
    }

    double update_mean = 0.0;
    double update_stdev = 0.0;
    calc_mean_stdev(update_flatness_[i], update_mean, update_stdev);
    params_.flatness_thr[i] = update_mean + update_stdev;

    int exceed_num = static_cast<int>(update_flatness_[i].size()) - params_.max_flatness_storage;
    if (exceed_num > 0) {
      update_flatness_[i].erase(update_flatness_[i].begin(),
                                update_flatness_[i].begin() + exceed_num);
    }
  }
}

void PatchWorkpp::reflected_noise_removal(Eigen::MatrixXf &cloud_in)
{
  if (cloud_in.cols() < 4) {
    std::cout << "RNR requires intensity information !" << std::endl;
    return;
  }

  int cnt = 0;
  for (int i = 0; i < cloud_in.rows(); i++) {
    const double r =
        std::sqrt(cloud_in.row(i)(0) * cloud_in.row(i)(0) + cloud_in.row(i)(1) * cloud_in.row(i)(1));
    const double z = cloud_in.row(i)(2);
    const double ver_angle_in_deg = std::atan2(z, r) * 180 / M_PI;

    if (ver_angle_in_deg < params_.RNR_ver_angle_thr && z < -params_.sensor_height - 0.8 &&
        cloud_in.row(i)(3) < params_.RNR_intensity_thr) {
      cloud_nonground_.push_back(
          PointXYZ(cloud_in.row(i)(0), cloud_in.row(i)(1), cloud_in.row(i)(2), i));
      cloud_in.row(i)(2) = std::numeric_limits<float>::min();
      cnt++;
    }
  }

  if (params_.verbose) {
    std::cout << "PatchWorkpp::reflected_noise_removal() - Number of Noises : " << cnt << std::endl;
  }
}

void PatchWorkpp::temporal_ground_revert(std::vector<double> ring_flatness,
                                         std::vector<patchwork::RevertCandidate> candidates,
                                         int concentric_idx)
{
  if (params_.verbose) {
    std::cout << "\033[1;34m"
              << "=========== Temporal Ground Revert (TGR) ==========="
              << "\033[0m" << std::endl;
  }

  double mean_flatness = 0.0;
  double stdev_flatness = 0.0;
  calc_mean_stdev(ring_flatness, mean_flatness, stdev_flatness);

  if (params_.verbose && !candidates.empty()) {
    std::cout << "[" << candidates[0].concentric_idx << ", " << candidates[0].sector_idx << "]"
              << " mean_flatness: " << mean_flatness << ", stdev_flatness: " << stdev_flatness
              << std::endl;
  }

  for (const auto &candidate : candidates) {
    // Debug
    if (params_.verbose) {
      std::cout << "\033[1;33m" << candidate.sector_idx << "th flat_sector_candidate"
                << " / flatness: " << candidate.ground_flatness
                << " / line_variable: " << candidate.line_variable
                << " / ground_num : " << candidate.regionwise_ground.size() << "\033[0m"
                << std::endl;
    }

    const double mu_flatness = mean_flatness + 1.5 * stdev_flatness;
    double prob_flatness =
        1 / (1 + std::exp((candidate.ground_flatness - mu_flatness) / (mu_flatness / 10)));

    if (candidate.regionwise_ground.size() > 1500 &&
        candidate.ground_flatness < params_.th_dist * params_.th_dist) {
      prob_flatness = 1.0;
    }

    double prob_line = 1.0;
    if (candidate.line_variable > 8.0) {
      prob_line = 0.0;
    }

    const bool revert = prob_line * prob_flatness > 0.5;

    if (concentric_idx < params_.num_rings_of_interest) {
      if (revert) {
        if (params_.verbose) {
          std::cout << "\033[1;32m"
                    << "REVERT TRUE"
                    << "\033[0m" << std::endl;
        }
        addCloud(cloud_ground_, candidate.regionwise_ground);
      } else {
        if (params_.verbose) {
          std::cout << "\033[1;31m"
                    << "FINAL REJECT"
                    << "\033[0m" << std::endl;
        }
        addCloud(cloud_nonground_, candidate.regionwise_ground);
      }
    }
  }

  if (params_.verbose) {
    std::cout << "\033[1;34m"
              << "===================================================="
              << "\033[0m" << std::endl;
  }
}

void PatchWorkpp::extract_piecewiseground(const int zone_idx,
                                          const std::vector<PointXYZ> &src,
                                          std::vector<PointXYZ> &dst,
                                          std::vector<PointXYZ> &non_ground_dst)
{
  // 0. Initialization
  if (!ground_pc_.empty()) {
    ground_pc_.clear();
  }
  if (!dst.empty()) {
    dst.clear();
  }
  if (!non_ground_dst.empty()) {
    non_ground_dst.clear();
  }

  // 1. Region-wise Vertical Plane Fitting (R-VPF)
  // : removes potential vertical plane under the ground plane
  std::vector<PointXYZ> src_wo_verticals = src;

  if (params_.enable_RVPF) {
    for (int i = 0; i < params_.num_iter; i++) {
      extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_, params_.th_seeds_v);
      estimate_plane(ground_pc_);

      if (zone_idx == 0 && normal_(2) < params_.uprightness_thr) {
        std::vector<PointXYZ> src_tmp = src_wo_verticals;
        src_wo_verticals.clear();

        for (const auto &point : src_tmp) {
          const double distance = calc_point_to_plane_d(point, normal_, d_);

          if (std::abs(distance) < params_.th_dist_v) {
            non_ground_dst.push_back(point);
          } else {
            src_wo_verticals.push_back(point);
          }
        }
      } else {
        break;
      }
    }
  }

  // 2. Region-wise Ground Plane Fitting (R-GPF)
  // : fits the ground plane

  extract_initial_seeds(zone_idx, src_wo_verticals, ground_pc_);
  estimate_plane(ground_pc_);

  // 远区自适应阈值：远处点云稀疏，适当放宽阈值
  const double th_dist_effective = (zone_idx >= 2) 
      ? params_.th_dist * params_.th_dist_far_scale 
      : params_.th_dist;

  for (int i = 0; i < params_.num_iter; i++) {
    ground_pc_.clear();

    for (const auto &point : src_wo_verticals) {
      const double distance = calc_point_to_plane_d(point, normal_, d_);

      if (i < params_.num_iter - 1) {
        if (distance < th_dist_effective) {
          ground_pc_.push_back(point);
        }
      } else {
        if (distance < th_dist_effective) {
          dst.push_back(point);
        } else {
          non_ground_dst.push_back(point);
        }
      }
    }

    if (i < params_.num_iter - 1) {
      estimate_plane(ground_pc_);
    } else {
      estimate_plane(dst);
    }
  }

  if (dst.size() + non_ground_dst.size() != src.size()) {
    std::cout << "\033[1;33m"
              << "Points are Missing/Adding !!! Please Check !! "
              << "\033[0m" << std::endl;
    std::cout << "gnd size: " << dst.size() << ", non gnd size: " << non_ground_dst.size()
              << ", src: " << src.size() << std::endl;
  }
}

double PatchWorkpp::calc_point_to_plane_d(PointXYZ p, Eigen::VectorXf normal, double d)
{
  return normal(0) * p.x + normal(1) * p.y + normal(2) * p.z + d;
}

void PatchWorkpp::calc_mean_stdev(const std::vector<double> &vec, double &mean, double &stdev)
{
  if (vec.size() <= 1) {
    return;
  }

  mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
  stdev = 0.0;
  for (double v : vec) {
    stdev += (v - mean) * (v - mean);
  }
  stdev /= vec.size() - 1;
  stdev = std::sqrt(stdev);
}

double PatchWorkpp::xy2theta(const double &x, const double &y)
{  // 0 ~ 2 * PI
  const double angle = std::atan2(y, x);
  return angle > 0 ? angle : 2 * M_PI + angle;
}

double PatchWorkpp::xy2radius(const double &x, const double &y)
{
  return std::sqrt(x * x + y * y);
}

void PatchWorkpp::pc2czm(const Eigen::MatrixXf &src, std::vector<Zone> &czm)
{
  const double max_range = params_.max_range;
  const double min_range = params_.min_range;
  const double min_range_0 = min_ranges_[0];
  const double min_range_1 = min_ranges_[1];
  const double min_range_2 = min_ranges_[2];
  const double min_range_3 = min_ranges_[3];
  const int num_ring_0 = params_.num_rings_each_zone[0];
  const int num_sector_0 = params_.num_sectors_each_zone[0];
  const int num_ring_1 = params_.num_rings_each_zone[1];
  const int num_sector_1 = params_.num_sectors_each_zone[1];
  const int num_ring_2 = params_.num_rings_each_zone[2];
  const int num_sector_2 = params_.num_sectors_each_zone[2];
  const int num_ring_3 = params_.num_rings_each_zone[3];
  const int num_sector_3 = params_.num_sectors_each_zone[3];

  for (int i = 0; i < src.rows(); i++) {
    const float x = src.row(i)(0);
    const float y = src.row(i)(1);
    const float z = src.row(i)(2);

    if (z == std::numeric_limits<float>::min()) {
      continue;
    }

    const double r = xy2radius(x, y);
    int ring_idx;
    int sector_idx;
    if ((r <= max_range) && (r > min_range)) {
      const double theta = xy2theta(x, y);

      if (r < min_range_1) {
        ring_idx = std::min(static_cast<int>(((r - min_range_0) / (ring_sizes_[0]))),
                            num_ring_0 - 1);
        sector_idx =
            std::min(static_cast<int>((theta / sector_sizes_[0])), num_sector_0 - 1);
        czm[0][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, i));
      } else if (r < min_range_2) {
        ring_idx = std::min(static_cast<int>(((r - min_range_1) / (ring_sizes_[1]))),
                            num_ring_1 - 1);
        sector_idx =
            std::min(static_cast<int>((theta / sector_sizes_[1])), num_sector_1 - 1);
        czm[1][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, i));
      } else if (r < min_range_3) {
        ring_idx = std::min(static_cast<int>(((r - min_range_2) / (ring_sizes_[2]))),
                            num_ring_2 - 1);
        sector_idx =
            std::min(static_cast<int>((theta / sector_sizes_[2])), num_sector_2 - 1);
        czm[2][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, i));
      } else {
        ring_idx = std::min(static_cast<int>(((r - min_range_3) / (ring_sizes_[3]))),
                            num_ring_3 - 1);
        sector_idx =
            std::min(static_cast<int>((theta / sector_sizes_[3])), num_sector_3 - 1);
        czm[3][ring_idx][sector_idx].emplace_back(PointXYZ(x, y, z, i));
      }

    } else {
      cloud_nonground_.push_back(PointXYZ(x, y, z, i));
    }
  }
  if (params_.verbose) {
    std::cout << "\033[1;33m"
              << "PatchWorkpp::pc2czm() - Divides pointcloud into the concentric zone model successfully"
              << "\033[0m" << std::endl;
  }
}

}  // namespace patchwork
