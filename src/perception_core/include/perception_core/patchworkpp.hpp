#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <Eigen/Dense>

#include <cmath>
#include <ctime>
#include <iostream>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace patchwork {

struct PointXYZ {
  float x;
  float y;
  float z;
  int idx;

  PointXYZ(float _x, float _y, float _z, int _idx = -1) : x(_x), y(_y), z(_z), idx(_idx) {}
};

struct RevertCandidate {
  int concentric_idx;
  int sector_idx;
  double ground_flatness;
  double line_variable;
  Eigen::VectorXf pc_mean;
  std::vector<PointXYZ> regionwise_ground;

  RevertCandidate(int _c_idx,
                  int _s_idx,
                  double _flatness,
                  double _line_var,
                  Eigen::VectorXf _pc_mean,
                  std::vector<PointXYZ> _ground)
      : concentric_idx(_c_idx),
        sector_idx(_s_idx),
        ground_flatness(_flatness),
        line_variable(_line_var),
        pc_mean(std::move(_pc_mean)),
        regionwise_ground(std::move(_ground)) {}
};

struct Params {
  bool verbose;
  bool enable_RNR;
  bool enable_RVPF;
  bool enable_TGR;

  int num_iter;
  int num_lpr;
  int num_min_pts;
  int num_zones;
  int num_rings_of_interest;

  double RNR_ver_angle_thr;
  double RNR_intensity_thr;

  double sensor_height;
  double th_seeds;
  double th_dist;
  double th_seeds_v;
  double th_dist_v;
  double max_range;
  double min_range;
  double uprightness_thr;
  double adaptive_seed_selection_margin;
  double intensity_thr;

  std::vector<int> num_sectors_each_zone;
  std::vector<int> num_rings_each_zone;

  int max_flatness_storage;
  int max_elevation_storage;

  std::vector<double> elevation_thr;
  std::vector<double> flatness_thr;

  Params();
};

class PatchWorkpp {
 public:
  using Ring = std::vector<std::vector<PointXYZ>>;
  using Zone = std::vector<Ring>;

  explicit PatchWorkpp(Params params);

  void estimateGround(Eigen::MatrixXf cloud_in);

  double getHeight() const { return params_.sensor_height; }
  double getTimeTaken() const { return time_taken_; }

  Eigen::MatrixX3f getGround() const { return toEigenCloud(cloud_ground_); }
  Eigen::MatrixX3f getNonground() const { return toEigenCloud(cloud_nonground_); }
  Eigen::VectorXi getGroundIndices() const { return toIndices(cloud_ground_); }
  Eigen::VectorXi getNongroundIndices() const { return toIndices(cloud_nonground_); }

  Eigen::MatrixX3f getCenters() const { return toEigenCloud(centers_); }
  Eigen::MatrixX3f getNormals() const { return toEigenCloud(normals_); }

 private:
  Params params_;

  time_t timer_ = 0;
  long time_taken_ = 0;

  std::vector<double> update_flatness_[4];
  std::vector<double> update_elevation_[4];

  double d_ = 0.0;

  Eigen::VectorXf normal_;
  Eigen::VectorXf singular_values_;
  Eigen::Matrix3f cov_;
  Eigen::VectorXf pc_mean_;

  std::vector<double> min_ranges_;
  std::vector<double> sector_sizes_;
  std::vector<double> ring_sizes_;

  std::vector<Zone> ConcentricZoneModel_;

  std::vector<PointXYZ> ground_pc_;
  std::vector<PointXYZ> non_ground_pc_;
  std::vector<PointXYZ> regionwise_ground_;
  std::vector<PointXYZ> regionwise_nonground_;

  std::vector<PointXYZ> cloud_ground_;
  std::vector<PointXYZ> cloud_nonground_;

  std::vector<PointXYZ> centers_;
  std::vector<PointXYZ> normals_;

  Eigen::MatrixX3f toEigenCloud(const std::vector<PointXYZ> &cloud) const;
  Eigen::VectorXi toIndices(const std::vector<PointXYZ> &cloud) const;

  void addCloud(std::vector<PointXYZ> &cloud, const std::vector<PointXYZ> &add);

  void flush_patches(std::vector<Zone> &czm);

  void pc2czm(const Eigen::MatrixXf &src, std::vector<Zone> &czm);

  void reflected_noise_removal(Eigen::MatrixXf &cloud_in);

  void temporal_ground_revert(std::vector<double> ring_flatness,
                              std::vector<patchwork::RevertCandidate> candidates,
                              int concentric_idx);

  double calc_point_to_plane_d(PointXYZ p, Eigen::VectorXf normal, double d);
  void calc_mean_stdev(const std::vector<double> &vec, double &mean, double &stdev);

  void update_elevation_thr();
  void update_flatness_thr();

  double xy2theta(const double &x, const double &y);

  double xy2radius(const double &x, const double &y);

  void estimate_plane(const std::vector<PointXYZ> &ground);

  void extract_piecewiseground(const int zone_idx,
                               const std::vector<PointXYZ> &src,
                               std::vector<PointXYZ> &dst,
                               std::vector<PointXYZ> &non_ground_dst);

  void extract_initial_seeds(const int zone_idx,
                             const std::vector<PointXYZ> &p_sorted,
                             std::vector<PointXYZ> &init_seeds);

  void extract_initial_seeds(const int zone_idx,
                             const std::vector<PointXYZ> &p_sorted,
                             std::vector<PointXYZ> &init_seeds,
                             double th_seed);
};

}  // namespace patchwork
