#include <perception_core/lidar_cluster_core.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <iostream>
#include <sstream>
#include <boost/make_shared.hpp>
#include <pcl/filters/crop_box.h>
#include <perception_core/fast_euclidean_clustering.hpp>

using std::string;
using std::vector;
using std::stod;

namespace {

struct VoxelKey
{
    int x;
    int y;
    int z;
};

struct VoxelKeyHash
{
    std::size_t operator()(const VoxelKey &key) const noexcept
    {
        std::size_t seed = std::hash<int>{}(key.x);
        seed ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct VoxelKeyEq
{
    bool operator()(const VoxelKey &a, const VoxelKey &b) const noexcept
    {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
};

struct VoxelAccum
{
    int count = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_i = 0.0;
};

VoxelKey ComputeVoxelKey(const PointType &p, double leaf_size)
{
    return VoxelKey{
        static_cast<int>(std::floor(p.x / leaf_size)),
        static_cast<int>(std::floor(p.y / leaf_size)),
        static_cast<int>(std::floor(p.z / leaf_size))};
}

void AdaptiveVoxelGrid(const pcl::PointCloud<PointType>::Ptr &in,
                       pcl::PointCloud<PointType>::Ptr &out,
                       double leaf_size,
                       int density_thr)
{
    if (!in || in->points.empty())
    {
        out->points.clear();
        out->width = 0;
        out->height = 1;
        out->is_dense = true;
        return;
    }
    if (leaf_size <= 0.0)
    {
        *out = *in;
        return;
    }

    std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash, VoxelKeyEq> voxels;
    voxels.reserve(in->points.size());
    for (const auto &p : in->points)
    {
        const VoxelKey key = ComputeVoxelKey(p, leaf_size);
        auto &accum = voxels[key];
        accum.count += 1;
        accum.sum_x += p.x;
        accum.sum_y += p.y;
        accum.sum_z += p.z;
        accum.sum_i += p.intensity;
    }

    out->points.clear();
    out->points.reserve(voxels.size());

    for (const auto &kv : voxels)
    {
        if (kv.second.count > density_thr)
        {
            // 高密度体素：降采样为质心
            PointType centroid;
            const double inv = 1.0 / static_cast<double>(kv.second.count);
            centroid.x = static_cast<float>(kv.second.sum_x * inv);
            centroid.y = static_cast<float>(kv.second.sum_y * inv);
            centroid.z = static_cast<float>(kv.second.sum_z * inv);
            centroid.intensity = static_cast<float>(kv.second.sum_i * inv);
            out->points.push_back(centroid);
        }
    }

    // 低密度体素：保留原始点
    for (const auto &p : in->points)
    {
        const VoxelKey key = ComputeVoxelKey(p, leaf_size);
        const auto it = voxels.find(key);
        if (it != voxels.end() && it->second.count <= density_thr)
        {
            out->points.push_back(p);
        }
    }

    out->width = out->points.size();
    out->height = 1;
    out->is_dense = in->is_dense;
}

// 距离自适应体素滤波 - 近处大体素，远处小体素
void DistanceAdaptiveVoxelFilter(
    const pcl::PointCloud<PointType>::Ptr &in,
    pcl::PointCloud<PointType>::Ptr &out,
    double near_leaf,       // 近距离体素大小
    double far_leaf,        // 远距离体素大小
    double dist_threshold)  // 距离阈值
{
    if (!in || in->points.empty())
    {
        out->points.clear();
        out->width = 0;
        out->height = 1;
        out->is_dense = true;
        return;
    }

    // 按距离分割点云
    pcl::PointCloud<PointType>::Ptr near_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr far_cloud(new pcl::PointCloud<PointType>);
    near_cloud->reserve(in->points.size());
    far_cloud->reserve(in->points.size() / 2);

    for (const auto &p : in->points)
    {
        float dist = std::sqrt(p.x * p.x + p.y * p.y);
        if (dist < dist_threshold)
        {
            near_cloud->push_back(p);
        }
        else
        {
            far_cloud->push_back(p);
        }
    }

    // 近距离：大体素降采样
    pcl::VoxelGrid<PointType> voxel;
    if (near_leaf > 0 && !near_cloud->empty())
    {
        voxel.setLeafSize(static_cast<float>(near_leaf),
                          static_cast<float>(near_leaf),
                          static_cast<float>(near_leaf));
        voxel.setInputCloud(near_cloud);
        voxel.filter(*near_cloud);
    }

    // 远距离：小体素或不降采样
    if (far_leaf > 0 && !far_cloud->empty())
    {
        voxel.setLeafSize(static_cast<float>(far_leaf),
                          static_cast<float>(far_leaf),
                          static_cast<float>(far_leaf));
        voxel.setInputCloud(far_cloud);
        voxel.filter(*far_cloud);
    }

    // 合并结果
    out->points.clear();
    out->points.reserve(near_cloud->size() + far_cloud->size());
    out->points.insert(out->points.end(), near_cloud->points.begin(), near_cloud->points.end());
    out->points.insert(out->points.end(), far_cloud->points.begin(), far_cloud->points.end());
    out->width = out->points.size();
    out->height = 1;
    out->is_dense = in->is_dense;
}

// 强度滤波 - 去除低强度噪声点
void IntensityFilter(
    pcl::PointCloud<PointType>::Ptr &cloud,
    float min_intensity)
{
    if (!cloud || cloud->points.empty())
    {
        return;
    }

    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
    filtered->reserve(cloud->points.size());

    for (const auto &p : cloud->points)
    {
        if (p.intensity >= min_intensity)
        {
            filtered->push_back(p);
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = cloud->is_dense;
    cloud.swap(filtered);
}

}  // namespace

float euc_distance(PointType a) { return float(sqrt(a.x * a.x + a.y * a.y)); }

void point_clip(const pcl::PointCloud<PointType>::Ptr in, double min_dist, double max_dist)
{
    pcl::ExtractIndices<PointType> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    float tmp_euc;
    for (size_t i = 0; i < in->points.size(); i++)
    {
        tmp_euc = euc_distance(in->points[i]);
        if (tmp_euc <= max_dist && in->points[i].x > 0 &&
            tmp_euc >= min_dist)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(false);
    cliper.filter(*in);
}

void lidar_cluster::Configure(const LidarClusterConfig &config)
{
    config_ = config;
    sensor_height_ = config.sensor_height;
    sensor_model_ = config.sensor_model;
    num_iter_ = config.ransac.num_iter;
    num_lpr_ = config.ransac.num_lpr;
    th_seeds_ = config.ransac.th_seeds;
    th_dist_ = config.ransac.th_dist;
    // RANSAC 优化参数
    enable_zone_ = config.ransac.enable_zone;
    zone_boundaries_ = config.ransac.zone_boundaries;
    adaptive_threshold_ = config.ransac.adaptive_threshold;
    th_dist_far_scale_ = config.ransac.th_dist_far_scale;
    min_normal_z_ = config.ransac.min_normal_z;
    progressive_iteration_ = config.ransac.progressive_iteration;
    max_range_ = roi_.custom.x_max > 0 ? roi_.custom.x_max : 50.0;
    road_type_ = config.road_type;
    ground_method_ = config.ground_method;
    roi_ = config.roi;
    filters_ = config.filters;
    cluster_ = config.cluster;
    roi_mode_ = roi_.mode;
    std::transform(roi_mode_.begin(),
                   roi_mode_.end(),
                   roi_mode_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    roi_use_point_clip_ = roi_.use_point_clip;
    z_up_ = roi_.z_max;
    z_down_ = roi_.z_min;
    vis_ = config.vis;
    str_range_ = config.str_range;
    str_seg_distance_ = config.str_seg_distance;

    min_height_ = config.min_height;
    max_height_ = config.max_height;
    min_area_ = config.min_area;
    max_area_ = config.max_area;
    max_box_altitude_ = config.max_box_altitude;

    // Configure confidence scorer
    perception::ConfidenceScorer::Config scorer_config;
    scorer_config.min_height = config.confidence.min_height;
    scorer_config.max_height = config.confidence.max_height;
    scorer_config.min_area = config.confidence.min_area;
    scorer_config.max_area = config.confidence.max_area;
    scorer_config.max_box_altitude = config.confidence.max_box_altitude;
    scorer_config.min_aspect_ratio = config.confidence.min_aspect_ratio;
    scorer_config.min_verticality = config.confidence.min_verticality;
    scorer_config.max_linearity = config.confidence.max_linearity;
    scorer_config.min_density_near = config.confidence.min_density_near;
    scorer_config.min_density_far = config.confidence.min_density_far;
    scorer_config.distance_threshold = config.confidence.distance_threshold;
    scorer_config.min_intensity_mean = config.confidence.min_intensity_mean;
    scorer_config.weight_size = config.confidence.weight_size;
    scorer_config.weight_shape = config.confidence.weight_shape;
    scorer_config.weight_density = config.confidence.weight_density;
    scorer_config.weight_intensity = config.confidence.weight_intensity;
    scorer_config.weight_position = config.confidence.weight_position;
    scorer_config.road_type = config.road_type;
    scorer_config.enable_model_fitting = config.confidence.enable_model_fitting;
    scorer_config.model_fit_bonus = config.confidence.model_fit_bonus;
    scorer_config.model_fit_penalty = config.confidence.model_fit_penalty;
    // Track semantic config
    scorer_config.track_semantic.enable = config.confidence.track_semantic.enable;
    scorer_config.track_semantic.weight = config.confidence.track_semantic.weight;
    scorer_config.track_semantic.expected_track_width = config.confidence.track_semantic.expected_track_width;
    scorer_config.track_semantic.expected_cone_spacing = config.confidence.track_semantic.expected_cone_spacing;
    scorer_config.track_semantic.spacing_tolerance = config.confidence.track_semantic.spacing_tolerance;
    scorer_config.track_semantic.width_tolerance = config.confidence.track_semantic.width_tolerance;
    scorer_config.track_semantic.isolation_radius = config.confidence.track_semantic.isolation_radius;
    scorer_config.track_semantic.min_neighbors_hard = config.confidence.track_semantic.min_neighbors_hard;
    scorer_config.track_semantic.max_isolation_distance = config.confidence.track_semantic.max_isolation_distance;
    confidence_scorer_.setConfig(scorer_config);

    // Configure cone tracker
    tracker_enabled_ = config.tracker.enable;
    if (tracker_enabled_) {
        perception::ConeTracker::Config tracker_config;
        tracker_config.association_threshold = config.tracker.association_threshold;
        tracker_config.confirm_frames = config.tracker.confirm_frames;
        tracker_config.delete_frames = config.tracker.delete_frames;
        tracker_config.process_noise = config.tracker.process_noise;
        tracker_config.measurement_noise = config.tracker.measurement_noise;
        tracker_config.only_output_confirmed = config.tracker.only_output_confirmed;
        tracker_config.confirmed_confidence_boost = config.tracker.confirmed_confidence_boost;
        cone_tracker_.setConfig(tracker_config);
    }

    // Configure topology repair
    topology_repair_.setConfig(config.topology);

    accel_x_max_ = roi_.accel.x_max;
    accel_x_min_ = roi_.accel.x_min;
    accel_y_max_ = roi_.accel.y_max;
    accel_y_min_ = roi_.accel.y_min;

    track_x_max_ = roi_.track.x_max;
    track_x_min_ = roi_.track.x_min;
    track_y_max_ = roi_.track.y_max;
    track_y_min_ = roi_.track.y_min;

    skid_x_max_ = roi_.skidpad.x_max;
    skid_x_min_ = roi_.skidpad.x_min;
    skid_y_max_ = roi_.skidpad.y_max;
    skid_y_min_ = roi_.skidpad.y_min;

    if (!roi_mode_.empty())
    {
        if (roi_mode_ == "skidpad")
        {
            road_type_ = 1;
        }
        else if (roi_mode_ == "accel")
        {
            road_type_ = 2;
        }
        else if (roi_mode_ == "track")
        {
            road_type_ = 3;
        }
        else if (roi_mode_ == "custom")
        {
            road_type_ = 3;
            accel_x_max_ = roi_.custom.x_max;
            accel_x_min_ = roi_.custom.x_min;
            accel_y_max_ = roi_.custom.y_max;
            accel_y_min_ = roi_.custom.y_min;
            track_x_max_ = roi_.custom.x_max;
            track_x_min_ = roi_.custom.x_min;
            track_y_max_ = roi_.custom.y_max;
            track_y_min_ = roi_.custom.y_min;
        }
    }

    // 使用新的聚类配置初始化距离分段
    dis_range = cluster_.distance_segments;
    seg_distances = cluster_.cluster_tolerance;

    // 兼容旧配置：如果新配置为空，使用字符串解析
    if (dis_range.empty() && !str_range_.empty())
    {
        splitString(str_range_, dis_range);
    }
    if (seg_distances.empty() && !str_seg_distance_.empty())
    {
        splitString(str_seg_distance_, seg_distances);
    }

    // 参数校验 - 确保 dis_range 和 seg_distances 有效
    if (dis_range.empty()) {
        std::cerr << "[Warning] distance_segments is empty, using defaults: 5,10,15,20" << std::endl;
        dis_range = {5.0, 10.0, 15.0, 20.0};
    }
    if (seg_distances.size() < dis_range.size() + 1) {
        std::cerr << "[Warning] cluster_tolerance size mismatch, using defaults" << std::endl;
        seg_distances = {0.15, 0.3, 0.5, 0.6, 0.6};
    }

    std::transform(ground_method_.begin(),
                   ground_method_.end(),
                   ground_method_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    patchwork_params_ = patchwork::Params();
    patchwork_params_.verbose = false;
    patchwork_params_.sensor_height = config.sensor_height;
    patchwork_params_.enable_RNR = config.patchworkpp.enable_rnr;
    patchwork_params_.enable_RVPF = config.patchworkpp.enable_rvpf;
    patchwork_params_.enable_TGR = config.patchworkpp.enable_tgr;
    patchwork_params_.num_iter = config.patchworkpp.num_iter;
    patchwork_params_.num_lpr = config.patchworkpp.num_lpr;
    patchwork_params_.num_min_pts = config.patchworkpp.num_min_pts;
    patchwork_params_.num_zones = config.patchworkpp.num_zones;
    patchwork_params_.num_rings_of_interest = config.patchworkpp.num_rings_of_interest;
    patchwork_params_.RNR_ver_angle_thr = config.patchworkpp.rnr_ver_angle_thr;
    patchwork_params_.RNR_intensity_thr = config.patchworkpp.rnr_intensity_thr;
    patchwork_params_.th_seeds = config.patchworkpp.th_seeds;
    patchwork_params_.th_dist = config.patchworkpp.th_dist;
    patchwork_params_.th_seeds_v = config.patchworkpp.th_seeds_v;
    patchwork_params_.th_dist_v = config.patchworkpp.th_dist_v;
    patchwork_params_.max_range = config.patchworkpp.max_range;
    patchwork_params_.min_range = config.patchworkpp.min_range;
    patchwork_params_.uprightness_thr = config.patchworkpp.uprightness_thr;
    patchwork_params_.adaptive_seed_selection_margin = config.patchworkpp.adaptive_seed_selection_margin;
    patchwork_params_.num_sectors_each_zone = config.patchworkpp.num_sectors_each_zone;
    patchwork_params_.num_rings_each_zone = config.patchworkpp.num_rings_each_zone;
    patchwork_params_.max_flatness_storage = config.patchworkpp.max_flatness_storage;
    patchwork_params_.max_elevation_storage = config.patchworkpp.max_elevation_storage;
    patchwork_params_.elevation_thr = config.patchworkpp.elevation_thr;
    patchwork_params_.flatness_thr = config.patchworkpp.flatness_thr;
    // Patchwork++ 优化参数
    patchwork_params_.th_dist_far_scale = config.patchworkpp.th_dist_far_scale;
    patchwork_params_.min_normal_z = config.patchworkpp.min_normal_z;
    patchwork_params_.far_zone_min_pts_scale = config.patchworkpp.far_zone_min_pts_scale;
    patchwork_.reset();
    if (ground_method_ == "patchworkpp") {
        patchwork_ = std::make_unique<patchwork::PatchWorkpp>(patchwork_params_);
    }
}

void lidar_cluster::init()
{
    getPointClouds = false;
    g_seeds_pc.reset(new pcl::PointCloud<PointType>());
    g_ground_pc.reset(new pcl::PointCloud<PointType>());
    g_not_ground_pc.reset(new pcl::PointCloud<PointType>());
    current_pc_ptr.reset(new pcl::PointCloud<PointType>());
    cloud_filtered.reset(new pcl::PointCloud<PointType>());
}

double lidar_cluster::getConfidence(PointType max, PointType min,
                                    Eigen::Vector4f centroid)
{
    double length = (double)std::fabs(max.x - min.x);
    double width = (double)std::fabs(max.y - min.y);
    double height = (double)std::fabs(max.z - min.z);

    // position of the cube, ignore the one that is not up
    if (length > height || width > height)
    {
        return -1;
    }

    double area = length * width;
    double score = 0.7;

    // #### remove bbox by limits
    // bbox lowest point altitude check
    if (std::fabs(min.z) > max_box_altitude_)
    {
        score = score - 2 * (min.z - max_box_altitude_);
    }

    // size measurement
    if (road_type_ == 1)
    {
        if (height > max_height_) score -= 0.05;
        if (area > max_area_) score -= 0.05;
    }
    else
    {
        if (height > max_height_) score -= 7 * (height - max_height_);
        if (area > max_area_) score -= 1.1 * (area - max_area_);
    }

    if (height < min_height_) score -= 0.1;
    if (area < min_area_) score -= 4 * (min_area_ - area);

    // centerness measurement
    return score;
}

double lidar_cluster::getConfidenceNew(const pcl::PointCloud<PointType>::Ptr& cluster)
{
    perception::ClusterFeatures features = feature_extractor_.extract(cluster);
    return confidence_scorer_.computeConfidence(features);
}

double lidar_cluster::getConfidenceWithFitting(const pcl::PointCloud<PointType>::Ptr& cluster)
{
    if (!cluster || cluster->points.empty()) {
        return 0.0;
    }

    perception::ClusterFeatures features = feature_extractor_.extract(cluster);
    return confidence_scorer_.computeConfidenceWithFitting(features, cluster);
}

void lidar_cluster::splitString(const std::string &in_string,
                                std::vector<double> &out_array)
{
    std::string tmp;
    std::istringstream in(in_string);
    while (std::getline(in, tmp, ','))
    {
        out_array.push_back(stod(tmp));
    }
}

// 直通滤波
void lidar_cluster::PassThrough(
    pcl::PointCloud<PointType>::Ptr &cloud_filtered)
{
    // 0. 强度滤波（可选，最先执行以去除弱反射噪声）
    if (filters_.intensity.enable)
    {
        IntensityFilter(cloud_filtered, filters_.intensity.min_intensity);
    }

    // 1. Z轴裁剪 + ROI裁剪（使用CropBox单次遍历，或传统PassThrough多次遍历）
    double x_min = 0.0, x_max = 0.0, y_min = 0.0, y_max = 0.0;

    // 根据road_type确定ROI范围
    if (road_type_ == 1)
    {
        // skidpad - 特殊处理point_clip
        if (roi_use_point_clip_)
        {
            // 先做Z轴裁剪
            pcl::PassThrough<PointType> pass;
            pass.setInputCloud(cloud_filtered);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(z_down_, z_up_);
            pass.filter(*cloud_filtered);
            // 然后用point_clip（使用配置参数）
            point_clip(cloud_filtered,
                       cluster_.point_clip.min_distance,
                       cluster_.point_clip.max_distance);
            // 跳过后续ROI裁剪
            goto voxel_filter;
        }
        x_min = skid_x_min_; x_max = skid_x_max_;
        y_min = skid_y_min_; y_max = skid_y_max_;
    }
    else if (road_type_ == 2)
    {
        x_min = accel_x_min_; x_max = accel_x_max_;
        y_min = accel_y_min_; y_max = accel_y_max_;
    }
    else
    {
        // track or custom (road_type_ == 3 or fallback)
        if (road_type_ != 3)
        {
            std::cerr << "[Warning] Received undefined road_type: " << road_type_
                      << ", falling back to track mode (road_type=3)" << std::endl;
            road_type_ = 3;
        }
        x_min = track_x_min_; x_max = track_x_max_;
        y_min = track_y_min_; y_max = track_y_max_;
    }

    // 使用CropBox单次遍历完成X/Y/Z裁剪（性能优化）
    if (filters_.use_cropbox)
    {
        pcl::CropBox<PointType> crop;
        crop.setMin(Eigen::Vector4f(static_cast<float>(x_min),
                                    static_cast<float>(y_min),
                                    static_cast<float>(z_down_), 1.0f));
        crop.setMax(Eigen::Vector4f(static_cast<float>(x_max),
                                    static_cast<float>(y_max),
                                    static_cast<float>(z_up_), 1.0f));
        crop.setInputCloud(cloud_filtered);
        crop.filter(*cloud_filtered);
    }
    else
    {
        // 传统PassThrough多次遍历
        pcl::PassThrough<PointType> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_down_, z_up_);
        pass.filter(*cloud_filtered);

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_min, x_max);
        pass.filter(*cloud_filtered);

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min, y_max);
        pass.filter(*cloud_filtered);
    }

voxel_filter:
    // 2. 体素降采样（减少后续处理点数）
    if (filters_.distance_adaptive_voxel.enable)
    {
        // 距离自适应体素滤波：近处大体素，远处小体素
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
        DistanceAdaptiveVoxelFilter(cloud_filtered,
                                    filtered,
                                    filters_.distance_adaptive_voxel.near_leaf,
                                    filters_.distance_adaptive_voxel.far_leaf,
                                    filters_.distance_adaptive_voxel.dist_threshold);
        cloud_filtered.swap(filtered);
    }
    else if (filters_.adaptive_voxel.enable)
    {
        pcl::PointCloud<PointType>::Ptr adaptive_filtered(new pcl::PointCloud<PointType>());
        AdaptiveVoxelGrid(cloud_filtered,
                          adaptive_filtered,
                          filters_.adaptive_voxel.leaf_size,
                          filters_.adaptive_voxel.density_thr);
        cloud_filtered.swap(adaptive_filtered);
    }
    else if (filters_.voxel.enable)
    {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setInputCloud(cloud_filtered);
        voxel.setLeafSize(static_cast<float>(filters_.voxel.leaf_size),
                          static_cast<float>(filters_.voxel.leaf_size),
                          static_cast<float>(filters_.voxel.leaf_size));
        voxel.filter(*cloud_filtered);
    }

    // 3. SOR离群点移除（在降采样后的点云上执行，更高效）
    if (filters_.sor.enable)
    {
        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setMeanK(filters_.sor.mean_k);
        sor.setStddevMulThresh(filters_.sor.stddev_mul);
        sor.filter(*cloud_filtered);
    }
}

// ROI裁剪 + 强度滤波（地面分割之前，保留原始点云密度）
void lidar_cluster::PassThroughROI(
    pcl::PointCloud<PointType>::Ptr &cloud_filtered)
{
    // 0. 强度滤波（可选，最先执行以去除弱反射噪声）
    if (filters_.intensity.enable)
    {
        IntensityFilter(cloud_filtered, filters_.intensity.min_intensity);
    }

    // 1. Z轴裁剪 + ROI裁剪
    double x_min = 0.0, x_max = 0.0, y_min = 0.0, y_max = 0.0;

    if (road_type_ == 1)
    {
        if (roi_use_point_clip_)
        {
            pcl::PassThrough<PointType> pass;
            pass.setInputCloud(cloud_filtered);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(z_down_, z_up_);
            pass.filter(*cloud_filtered);
            point_clip(cloud_filtered,
                       cluster_.point_clip.min_distance,
                       cluster_.point_clip.max_distance);
            return;
        }
        x_min = skid_x_min_; x_max = skid_x_max_;
        y_min = skid_y_min_; y_max = skid_y_max_;
    }
    else if (road_type_ == 2)
    {
        x_min = accel_x_min_; x_max = accel_x_max_;
        y_min = accel_y_min_; y_max = accel_y_max_;
    }
    else
    {
        if (road_type_ != 3)
        {
            std::cerr << "[Warning] Received undefined road_type: " << road_type_
                      << ", falling back to track mode (road_type=3)" << std::endl;
            road_type_ = 3;
        }
        x_min = track_x_min_; x_max = track_x_max_;
        y_min = track_y_min_; y_max = track_y_max_;
    }

    if (filters_.use_cropbox)
    {
        pcl::CropBox<PointType> crop;
        crop.setMin(Eigen::Vector4f(static_cast<float>(x_min),
                                    static_cast<float>(y_min),
                                    static_cast<float>(z_down_), 1.0f));
        crop.setMax(Eigen::Vector4f(static_cast<float>(x_max),
                                    static_cast<float>(y_max),
                                    static_cast<float>(z_up_), 1.0f));
        crop.setInputCloud(cloud_filtered);
        crop.filter(*cloud_filtered);
    }
    else
    {
        pcl::PassThrough<PointType> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_down_, z_up_);
        pass.filter(*cloud_filtered);

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_min, x_max);
        pass.filter(*cloud_filtered);

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min, y_max);
        pass.filter(*cloud_filtered);
    }
}

// 降采样 + SOR（地面分割之后，仅对非地面点）
// 目的：减少聚类计算量，同时不影响地面分割精度
void lidar_cluster::PostGroundFilter(
    pcl::PointCloud<PointType>::Ptr &cloud)
{
    if (!cloud || cloud->points.empty()) return;

    // 0. 柱状障碍物滤波：基于局部z跨度去除树/墙壁
    //    将xy平面划分为2D栅格，z跨度过大的栅格视为高大障碍物
    if (filters_.obstacle_height.enable)
    {
        const double gs = filters_.obstacle_height.grid_size;
        const double max_span = filters_.obstacle_height.max_z_span;
        const int min_pts = filters_.obstacle_height.min_points_to_judge;
        const float min_dist_sq = static_cast<float>(
            filters_.obstacle_height.min_distance * filters_.obstacle_height.min_distance);
        const double inv_gs = 1.0 / gs;

        // 栅格统计：z_min, z_max, count（仅统计远处点）
        struct CellStat {
            float z_min = std::numeric_limits<float>::max();
            float z_max = std::numeric_limits<float>::lowest();
            int count = 0;
        };
        std::unordered_map<int64_t, CellStat> grid;
        grid.reserve(cloud->points.size() / 2);

        // 编码栅格key：将(gx, gy)打包到int64
        auto encode = [&](float x, float y) -> int64_t {
            int gx = static_cast<int>(std::floor(x * inv_gs));
            int gy = static_cast<int>(std::floor(y * inv_gs));
            return (static_cast<int64_t>(gx) << 32) | static_cast<int64_t>(static_cast<uint32_t>(gy));
        };

        // 第一遍：仅对远处点统计栅格z范围
        for (const auto &p : cloud->points)
        {
            float dist_sq = p.x * p.x + p.y * p.y;
            if (dist_sq < min_dist_sq) continue;  // 近处跳过
            int64_t key = encode(p.x, p.y);
            auto &cell = grid[key];
            if (p.z < cell.z_min) cell.z_min = p.z;
            if (p.z > cell.z_max) cell.z_max = p.z;
            cell.count++;
        }

        // 第二遍：近处点全部保留，远处点过滤z跨度过大的栅格
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
        filtered->reserve(cloud->points.size());
        for (const auto &p : cloud->points)
        {
            float dist_sq = p.x * p.x + p.y * p.y;
            if (dist_sq < min_dist_sq)
            {
                filtered->push_back(p);  // 近处无条件保留
                continue;
            }
            int64_t key = encode(p.x, p.y);
            const auto &cell = grid[key];
            if (cell.count < min_pts ||
                (cell.z_max - cell.z_min) <= max_span)
            {
                filtered->push_back(p);
            }
        }
        filtered->width = filtered->points.size();
        filtered->height = 1;
        filtered->is_dense = cloud->is_dense;
        cloud.swap(filtered);
    }

    // 1. 体素降采样
    if (filters_.distance_adaptive_voxel.enable)
    {
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
        DistanceAdaptiveVoxelFilter(cloud,
                                    filtered,
                                    filters_.distance_adaptive_voxel.near_leaf,
                                    filters_.distance_adaptive_voxel.far_leaf,
                                    filters_.distance_adaptive_voxel.dist_threshold);
        cloud.swap(filtered);
    }
    else if (filters_.adaptive_voxel.enable)
    {
        pcl::PointCloud<PointType>::Ptr adaptive_filtered(new pcl::PointCloud<PointType>());
        AdaptiveVoxelGrid(cloud,
                          adaptive_filtered,
                          filters_.adaptive_voxel.leaf_size,
                          filters_.adaptive_voxel.density_thr);
        cloud.swap(adaptive_filtered);
    }
    else if (filters_.voxel.enable)
    {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(static_cast<float>(filters_.voxel.leaf_size),
                          static_cast<float>(filters_.voxel.leaf_size),
                          static_cast<float>(filters_.voxel.leaf_size));
        voxel.filter(*cloud);
    }

    // 2. SOR离群点移除
    if (filters_.sor.enable)
    {
        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(filters_.sor.mean_k);
        sor.setStddevMulThresh(filters_.sor.stddev_mul);
        sor.filter(*cloud);
    }
}

void lidar_cluster::EucClusterMethod(
    pcl::PointCloud<PointType>::Ptr inputcloud,
    std::vector<pcl::PointIndices> &cluster_indices,
    const double &max_cluster_dis)
{
    if (!inputcloud || inputcloud->points.empty())
    {
        return;
    }
    pcl::search::KdTree<PointType>::Ptr tree(
        new pcl::search::KdTree<PointType>);
    tree->setInputCloud(inputcloud);

    if (cluster_.fec.enable)
    {
        perception::FastEuclideanClustering<PointType> fec;
        fec.setClusterTolerance(max_cluster_dis);
        fec.setMinClusterSize(cluster_.min_cluster_size);
        fec.setMaxClusterSize(cluster_.max_cluster_size);
        fec.setQuality(cluster_.fec.quality);
        fec.setSearchMethod(tree);
        fec.setInputCloud(inputcloud);
        fec.segment(cluster_indices);
    }
    else
    {
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(max_cluster_dis);
        ec.setMinClusterSize(cluster_.min_cluster_size);
        ec.setMaxClusterSize(cluster_.max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(inputcloud);
        ec.extract(cluster_indices);
    }
}

// 带自适应聚类大小的欧式聚类
void lidar_cluster::EucClusterMethodAdaptive(
    pcl::PointCloud<PointType>::Ptr inputcloud,
    std::vector<pcl::PointIndices> &cluster_indices,
    const double &max_cluster_dis,
    int min_cluster_size,
    int max_cluster_size)
{
    if (!inputcloud || inputcloud->points.empty())
    {
        return;
    }
    pcl::search::KdTree<PointType>::Ptr tree(
        new pcl::search::KdTree<PointType>);
    tree->setInputCloud(inputcloud);

    if (cluster_.fec.enable)
    {
        perception::FastEuclideanClustering<PointType> fec;
        fec.setClusterTolerance(max_cluster_dis);
        fec.setMinClusterSize(min_cluster_size);
        fec.setMaxClusterSize(max_cluster_size);
        fec.setQuality(cluster_.fec.quality);
        fec.setSearchMethod(tree);
        fec.setInputCloud(inputcloud);
        fec.segment(cluster_indices);
    }
    else
    {
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(max_cluster_dis);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(inputcloud);
        ec.extract(cluster_indices);
    }
}

// 计算自适应聚类大小（根据距离线性插值）
void lidar_cluster::getAdaptiveClusterSize(
    double distance,
    int &min_size,
    int &max_size) const
{
    if (!cluster_.adaptive_size.enable)
    {
        min_size = cluster_.min_cluster_size;
        max_size = cluster_.max_cluster_size;
        return;
    }

    const auto &adaptive = cluster_.adaptive_size;
    double near_dist = dis_range.empty() ? 5.0 : dis_range.front();
    double far_dist = dis_range.empty() ? 20.0 : dis_range.back();

    if (distance <= near_dist)
    {
        // 近距离：使用近距离参数
        min_size = adaptive.near_min_size;
        max_size = adaptive.near_max_size;
    }
    else if (distance >= far_dist)
    {
        // 远距离：使用远距离参数
        min_size = adaptive.far_min_size;
        max_size = adaptive.far_max_size;
    }
    else
    {
        // 中间距离：线性插值
        double ratio = (distance - near_dist) / (far_dist - near_dist);
        min_size = static_cast<int>(adaptive.near_min_size +
                                    ratio * (adaptive.far_min_size - adaptive.near_min_size));
        max_size = static_cast<int>(adaptive.near_max_size +
                                    ratio * (adaptive.far_max_size - adaptive.near_max_size));
        // 确保最小值有效
        min_size = std::max(1, min_size);
        max_size = std::max(min_size, max_size);
    }
}

// 优化版本：使用索引分段，避免点云复制
void lidar_cluster::EuclideanAdaptiveClusterMethod(
    pcl::PointCloud<PointType>::Ptr inputcloud,
    std::vector<std::vector<pcl::PointIndices>> &cluster_indices,
    std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array)
{
    if (!inputcloud || inputcloud->points.empty())
    {
        return;
    }

    size_t num_segments = cloud_segments_array.size();

    // 第一遍：统计每个分段的点数，用于预分配
    std::vector<size_t> segment_counts(num_segments, 0);
    for (size_t i = 0; i < inputcloud->points.size(); i++)
    {
        const auto &p = inputcloud->points[i];
        float origin_dis = std::sqrt(p.x * p.x + p.y * p.y);
        size_t seg_idx = num_segments - 1;  // 默认最后一段
        for (size_t j = 0; j < dis_range.size(); ++j)
        {
            if (origin_dis < dis_range[j])
            {
                seg_idx = j;
                break;
            }
        }
        segment_counts[seg_idx]++;
    }

    // 预分配内存
    for (size_t i = 0; i < cloud_segments_array.size(); i++)
    {
        cloud_segments_array[i]->points.reserve(segment_counts[i]);
    }

    // 第二遍：分配点到各分段（已预分配，push_back不会触发重分配）
    for (size_t i = 0; i < inputcloud->points.size(); i++)
    {
        const auto &p = inputcloud->points[i];
        float origin_dis = std::sqrt(p.x * p.x + p.y * p.y);
        size_t seg_idx = num_segments - 1;
        for (size_t j = 0; j < dis_range.size(); ++j)
        {
            if (origin_dis < dis_range[j])
            {
                seg_idx = j;
                break;
            }
        }
        cloud_segments_array[seg_idx]->points.push_back(p);
    }

    // 更新点云元数据
    for (size_t i = 0; i < cloud_segments_array.size(); i++)
    {
        cloud_segments_array[i]->width = cloud_segments_array[i]->points.size();
        cloud_segments_array[i]->height = 1;
        cloud_segments_array[i]->is_dense = inputcloud->is_dense;
    }

    // 对每个分段进行聚类（使用自适应聚类大小）
    cluster_indices.resize(cloud_segments_array.size());
    for (size_t i = 0; i < cloud_segments_array.size(); i++)
    {
        if (!cloud_segments_array[i]->points.empty())
        {
            // 计算该分段的代表距离（使用分段中点）
            double segment_dist;
            if (i == 0)
            {
                segment_dist = dis_range[0] / 2.0;  // 第一段：0 到 dis_range[0]
            }
            else if (i < dis_range.size())
            {
                segment_dist = (dis_range[i - 1] + dis_range[i]) / 2.0;
            }
            else
            {
                segment_dist = dis_range.back() + 5.0;  // 最后一段：> dis_range.back()
            }

            // 获取自适应聚类大小
            int min_size, max_size;
            getAdaptiveClusterSize(segment_dist, min_size, max_size);

            // 使用自适应参数进行聚类
            EucClusterMethodAdaptive(cloud_segments_array[i], cluster_indices[i],
                                     seg_distances[i], min_size, max_size);
        }
    }
}

// 计算自适应 DBSCAN eps（根据距离线性插值）
double lidar_cluster::getAdaptiveEps(double distance) const
{
    if (!cluster_.dbscan.adaptive_eps)
    {
        return cluster_.dbscan.eps;
    }

    double near_dist = dis_range.empty() ? 5.0 : dis_range.front();
    double far_dist = dis_range.empty() ? 20.0 : dis_range.back();

    if (distance <= near_dist)
    {
        return cluster_.dbscan.eps_near;
    }
    else if (distance >= far_dist)
    {
        return cluster_.dbscan.eps_far;
    }
    else
    {
        // 线性插值
        double ratio = (distance - near_dist) / (far_dist - near_dist);
        return cluster_.dbscan.eps_near + ratio * (cluster_.dbscan.eps_far - cluster_.dbscan.eps_near);
    }
}

// DBSCAN 聚类实现
void lidar_cluster::DbscanClusterMethod(
    pcl::PointCloud<PointType>::Ptr inputcloud,
    std::vector<pcl::PointIndices> &cluster_indices,
    double eps,
    int min_pts,
    int max_cluster_size)
{
    if (!inputcloud || inputcloud->points.empty())
    {
        return;
    }

    size_t n_points = inputcloud->points.size();

    // 构建 KdTree
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(inputcloud);

    // 点状态：0=未访问，1=噪声，2+=聚类ID
    std::vector<int> point_labels(n_points, 0);
    int current_cluster = 0;

    for (size_t i = 0; i < n_points; ++i)
    {
        if (point_labels[i] != 0)
        {
            continue;  // 已处理
        }

        // 查找邻域
        std::vector<int> neighbors;
        std::vector<float> distances;
        tree->radiusSearch(inputcloud->points[i], eps, neighbors, distances);

        if (static_cast<int>(neighbors.size()) < min_pts)
        {
            point_labels[i] = -1;  // 标记为噪声
            continue;
        }

        // 开始新聚类
        current_cluster++;
        point_labels[i] = current_cluster;

        // 扩展聚类
        std::vector<int> seed_queue = neighbors;
        size_t queue_idx = 0;

        while (queue_idx < seed_queue.size())
        {
            int neighbor_idx = seed_queue[queue_idx++];

            if (point_labels[neighbor_idx] == -1)
            {
                // 噪声点变为边界点
                point_labels[neighbor_idx] = current_cluster;
            }

            if (point_labels[neighbor_idx] != 0)
            {
                continue;  // 已属于某个聚类
            }

            point_labels[neighbor_idx] = current_cluster;

            // 查找该点的邻域
            std::vector<int> sub_neighbors;
            std::vector<float> sub_distances;
            tree->radiusSearch(inputcloud->points[neighbor_idx], eps, sub_neighbors, sub_distances);

            if (static_cast<int>(sub_neighbors.size()) >= min_pts)
            {
                // 核心点，扩展邻域
                for (int sub_idx : sub_neighbors)
                {
                    if (point_labels[sub_idx] == 0 || point_labels[sub_idx] == -1)
                    {
                        seed_queue.push_back(sub_idx);
                    }
                }
            }
        }
    }

    // 收集聚类结果
    std::unordered_map<int, std::vector<int>> cluster_map;
    for (size_t i = 0; i < n_points; ++i)
    {
        if (point_labels[i] > 0)
        {
            cluster_map[point_labels[i]].push_back(static_cast<int>(i));
        }
    }

    // 转换为 PointIndices 格式，过滤过大的聚类
    cluster_indices.clear();
    cluster_indices.reserve(cluster_map.size());
    for (const auto &kv : cluster_map)
    {
        if (static_cast<int>(kv.second.size()) <= max_cluster_size)
        {
            pcl::PointIndices indices;
            indices.indices = kv.second;
            cluster_indices.push_back(std::move(indices));
        }
    }
}

// DBSCAN 自适应聚类（按距离分段）
void lidar_cluster::DbscanAdaptiveClusterMethod(
    pcl::PointCloud<PointType>::Ptr inputcloud,
    std::vector<std::vector<pcl::PointIndices>> &cluster_indices,
    std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array)
{
    if (!inputcloud || inputcloud->points.empty())
    {
        return;
    }

    // 第一遍：统计每个分段的点数
    std::vector<size_t> segment_counts(cloud_segments_array.size(), 0);
    for (size_t i = 0; i < inputcloud->points.size(); i++)
    {
        const auto &p = inputcloud->points[i];
        float origin_dis = std::sqrt(p.x * p.x + p.y * p.y);
        size_t seg_idx = cloud_segments_array.size() - 1;
        for (size_t j = 0; j < dis_range.size(); ++j)
        {
            if (origin_dis < dis_range[j])
            {
                seg_idx = j;
                break;
            }
        }
        segment_counts[seg_idx]++;
    }

    // 预分配内存
    for (size_t i = 0; i < cloud_segments_array.size(); i++)
    {
        cloud_segments_array[i]->points.reserve(segment_counts[i]);
    }

    // 第二遍：分配点到各分段
    for (size_t i = 0; i < inputcloud->points.size(); i++)
    {
        const auto &p = inputcloud->points[i];
        float origin_dis = std::sqrt(p.x * p.x + p.y * p.y);
        size_t seg_idx = cloud_segments_array.size() - 1;
        for (size_t j = 0; j < dis_range.size(); ++j)
        {
            if (origin_dis < dis_range[j])
            {
                seg_idx = j;
                break;
            }
        }
        cloud_segments_array[seg_idx]->points.push_back(p);
    }

    // 更新点云元数据
    for (size_t i = 0; i < cloud_segments_array.size(); i++)
    {
        cloud_segments_array[i]->width = cloud_segments_array[i]->points.size();
        cloud_segments_array[i]->height = 1;
        cloud_segments_array[i]->is_dense = inputcloud->is_dense;
    }

    // 对每个分段进行 DBSCAN 聚类
    cluster_indices.resize(cloud_segments_array.size());
    for (size_t i = 0; i < cloud_segments_array.size(); i++)
    {
        if (!cloud_segments_array[i]->points.empty())
        {
            // 计算该分段的代表距离
            double segment_dist;
            if (i == 0)
            {
                segment_dist = dis_range[0] / 2.0;
            }
            else if (i < dis_range.size())
            {
                segment_dist = (dis_range[i - 1] + dis_range[i]) / 2.0;
            }
            else
            {
                segment_dist = dis_range.back() + 5.0;
            }

            // 获取自适应参数
            double eps = getAdaptiveEps(segment_dist);
            int min_size, max_size;
            getAdaptiveClusterSize(segment_dist, min_size, max_size);

            // DBSCAN 聚类
            DbscanClusterMethod(cloud_segments_array[i], cluster_indices[i],
                                eps, cluster_.dbscan.min_pts, max_size);
        }
    }
}

void lidar_cluster::clusterMethod32(LidarClusterOutput *output)
{
    last_cluster_count_ = 0;
    if (!output)
    {
        return;
    }

    output->cones.clear();
    output->non_cones.clear();
    // 复用已有点云对象，避免每帧重新分配
    if (!output->cones_cloud) {
        output->cones_cloud.reset(new pcl::PointCloud<PointType>);
    }
    output->cones_cloud->points.clear();

    std::vector<std::vector<pcl::PointIndices>> cluster_indices;

    // 动态确定分段数量
    size_t num_segments = dis_range.size() + 1;
    std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array(num_segments);
    for (size_t i = 0; i < cloud_segments_array.size(); i++)
    {
        cloud_segments_array[i].reset(new pcl::PointCloud<PointType>);
    }

    // 根据配置选择聚类方法
    if (cluster_.method == "dbscan")
    {
        DbscanAdaptiveClusterMethod(g_not_ground_pc, cluster_indices, cloud_segments_array);
    }
    else
    {
        // 默认使用欧式聚类
        EuclideanAdaptiveClusterMethod(g_not_ground_pc, cluster_indices, cloud_segments_array);
    }

    // 统计总聚类数
    size_t total_clusters = 0;
    for (const auto &segment : cluster_indices)
    {
        total_clusters += segment.size();
    }

    // 预估锥桶数量，预分配内存
    output->cones.reserve(total_clusters);
    if (vis_)
    {
        output->non_cones.reserve(total_clusters);
    }

    // 预估最终点云大小（假设平均每个聚类10个点）
    output->cones_cloud->points.reserve(total_clusters * 10);

    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        for (const auto &indices : cluster_indices[i])
        {
            pcl::PointCloud<PointType>::Ptr cloud_cluster(
                new pcl::PointCloud<PointType>);
            cloud_cluster->points.reserve(indices.indices.size());

            for (int idx : indices.indices)
            {
                cloud_cluster->points.push_back(
                    cloud_segments_array[i]->points[idx]);
            }

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            PointType _min;
            PointType _max;
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            pcl::getMinMax3D(*cloud_cluster, _min, _max);

            float euc = std::sqrt(centroid[0] * centroid[0] +
                                  centroid[1] * centroid[1]);

            // 距离自适应Y轴梯形ROI拒绝（在confidence计算之前，节省计算量）
            bool y_rejected = false;
            if (config_.roi.adaptive_y.enable)
            {
                const auto &ay = config_.roi.adaptive_y;
                double x_pos = static_cast<double>(centroid[0]);
                double x_max_roi = (road_type_ == 2) ? accel_x_max_ : track_x_max_;
                double y_limit;
                if (x_pos <= ay.ramp_start_x)
                {
                    y_limit = ay.near_y_half;
                }
                else if (x_pos >= x_max_roi)
                {
                    y_limit = ay.far_y_half;
                }
                else
                {
                    double ratio = (x_pos - ay.ramp_start_x) / (x_max_roi - ay.ramp_start_x);
                    y_limit = ay.near_y_half + ratio * (ay.far_y_half - ay.near_y_half);
                }
                if (std::fabs(centroid[1]) > y_limit)
                {
                    y_rejected = true;
                }
            }

            double confidence = y_rejected ? -1.0 : getConfidenceWithFitting(cloud_cluster);

            // 距离自适应置信度门槛（替代原来的 confidence > 0）
            double min_conf;
            {
                const auto &cc = config_.confidence;
                if (euc <= cc.confidence_ramp_start)
                {
                    min_conf = cc.min_confidence_near;
                }
                else if (euc >= cc.confidence_ramp_end)
                {
                    min_conf = cc.min_confidence_far;
                }
                else
                {
                    double t = (euc - cc.confidence_ramp_start) /
                               (cc.confidence_ramp_end - cc.confidence_ramp_start);
                    min_conf = cc.min_confidence_near + t * (cc.min_confidence_far - cc.min_confidence_near);
                }
            }
            bool is_cone = (confidence > min_conf);

            ConeDetection det;
            det.min = pcl::PointXYZ(_min.x, _min.y, _min.z);
            det.max = pcl::PointXYZ(_max.x, _max.y, _max.z);
            det.centroid = pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
            det.confidence = confidence;
            det.distance = euc;
            det.cluster = cloud_cluster;
            det.is_cone = is_cone;

            if (is_cone)
            {
                output->cones.push_back(std::move(det));
                // 优化：直接insert代替operator+（避免临时对象和多次拷贝）
                output->cones_cloud->points.insert(
                    output->cones_cloud->points.end(),
                    cloud_cluster->points.begin(),
                    cloud_cluster->points.end());
            }
            else if (vis_)
            {
                output->non_cones.push_back(std::move(det));
            }
        }
    }

    output->cones_cloud->width = output->cones_cloud->points.size();
    output->cones_cloud->height = 1;
    output->cones_cloud->is_dense = g_not_ground_pc->is_dense;
    last_cluster_count_ = total_clusters;

    // Two-pass re-scoring with track semantic context (Phase 4 / Innovation I1)
    if (config_.confidence.track_semantic.enable &&
        config_.confidence.track_semantic.weight > 0.0 &&
        output->cones.size() >= 2) {

        // Collect all cone centroids
        std::vector<pcl::PointXYZ> all_centroids;
        all_centroids.reserve(output->cones.size());
        for (const auto& det : output->cones) {
            all_centroids.push_back(det.centroid);
        }

        // Re-score each cone with neighbor context
        for (size_t i = 0; i < output->cones.size(); ++i) {
            auto& det = output->cones[i];
            if (det.cluster) {
                perception::ClusterFeatures features = feature_extractor_.extract(det.cluster);
                det.confidence = confidence_scorer_.computeConfidenceWithContext(
                    features, det.cluster, all_centroids, static_cast<int>(i));
            }
        }

        // Re-filter with distance-adaptive threshold
        std::vector<ConeDetection> filtered;
        filtered.reserve(output->cones.size());
        pcl::PointCloud<PointType>::Ptr new_cloud(new pcl::PointCloud<PointType>);

        for (auto& det : output->cones) {
            double euc = det.distance;
            double min_conf;
            const auto &cc = config_.confidence;
            if (euc <= cc.confidence_ramp_start) {
                min_conf = cc.min_confidence_near;
            } else if (euc >= cc.confidence_ramp_end) {
                min_conf = cc.min_confidence_far;
            } else {
                double t = (euc - cc.confidence_ramp_start) /
                           (cc.confidence_ramp_end - cc.confidence_ramp_start);
                min_conf = cc.min_confidence_near + t * (cc.min_confidence_far - cc.min_confidence_near);
            }

            if (det.confidence > min_conf) {
                if (det.cluster) {
                    new_cloud->points.insert(
                        new_cloud->points.end(),
                        det.cluster->points.begin(),
                        det.cluster->points.end());
                }
                filtered.push_back(std::move(det));
            }
        }

        output->cones = std::move(filtered);
        new_cloud->width = new_cloud->points.size();
        new_cloud->height = 1;
        new_cloud->is_dense = g_not_ground_pc->is_dense;
        output->cones_cloud = new_cloud;
    }
}

void lidar_cluster::clusterMethod16(LidarClusterOutput *output)
{
    last_cluster_count_ = 0;
    if (!output)
    {
        return;
    }

    output->cones.clear();
    output->non_cones.clear();
    // 复用已有点云对象，避免每帧重新分配
    if (!output->cones_cloud) {
        output->cones_cloud.reset(new pcl::PointCloud<PointType>);
    }
    output->cones_cloud->points.clear();

    std::vector<pcl::PointIndices> cluster_indices;

    EucClusterMethod(g_not_ground_pc, cluster_indices, cluster_.vlp16.cluster_tolerance);
    size_t total_clusters = cluster_indices.size();

    // 预分配内存
    output->cones.reserve(total_clusters);
    if (vis_)
    {
        output->non_cones.reserve(total_clusters);
    }
    output->cones_cloud->points.reserve(total_clusters * 10);

    for (const auto &indices : cluster_indices)
    {
        pcl::PointCloud<PointType>::Ptr cloud_cluster(
            new pcl::PointCloud<PointType>);
        cloud_cluster->points.reserve(indices.indices.size());

        for (int idx : indices.indices)
        {
            cloud_cluster->points.push_back(g_not_ground_pc->points[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = g_not_ground_pc->is_dense;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);

        PointType min;
        PointType max;
        pcl::getMinMax3D(*cloud_cluster, min, max);
        double xx = std::fabs(max.x - min.x);
        double yy = std::fabs(max.y - min.y);
        double zz = std::fabs(max.z - min.z);

        bool is_cone = (xx < cluster_.vlp16.max_bbox_x &&
                        yy < cluster_.vlp16.max_bbox_y &&
                        zz < cluster_.vlp16.max_bbox_z);
        float euc = std::sqrt(centroid[0] * centroid[0] +
                              centroid[1] * centroid[1]);

        ConeDetection det;
        det.min = pcl::PointXYZ(min.x, min.y, min.z);
        det.max = pcl::PointXYZ(max.x, max.y, max.z);
        det.centroid = pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
        det.confidence = 0.0;
        det.distance = euc;
        det.cluster = cloud_cluster;
        det.is_cone = is_cone;

        if (is_cone)
        {
            output->cones.push_back(std::move(det));
            // 优化：直接insert代替operator+
            output->cones_cloud->points.insert(
                output->cones_cloud->points.end(),
                cloud_cluster->points.begin(),
                cloud_cluster->points.end());
        }
        else if (vis_)
        {
            output->non_cones.push_back(std::move(det));
        }
    }

    output->cones_cloud->width = output->cones_cloud->points.size();
    output->cones_cloud->height = 1;
    output->cones_cloud->is_dense = g_not_ground_pc->is_dense;
    last_cluster_count_ = total_clusters;
}

void lidar_cluster::AccumulateFrames(pcl::PointCloud<PointType>::Ptr &cloud)
{
    if (!cluster_.multi_frame.enable || !cloud || cloud->points.empty())
    {
        return;
    }

    const double max_dist = cluster_.multi_frame.max_distance;
    const double max_dist_sq = max_dist * max_dist;
    const int num_frames = cluster_.multi_frame.num_frames;

    // 分离远处点
    pcl::PointCloud<PointType>::Ptr far_points(new pcl::PointCloud<PointType>);
    far_points->reserve(cloud->points.size() / 4);

    for (const auto &p : cloud->points)
    {
        double dist_sq = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y;
        if (dist_sq > max_dist_sq)
        {
            far_points->push_back(p);
        }
    }

    // 存入缓冲区
    frame_buffer_.push_back(far_points);
    while (static_cast<int>(frame_buffer_.size()) > num_frames)
    {
        frame_buffer_.pop_front();
    }

    // 将历史帧的远处点合并到当前点云
    if (frame_buffer_.size() > 1)
    {
        // 预估总点数
        size_t extra_points = 0;
        for (size_t i = 0; i < frame_buffer_.size() - 1; ++i)
        {
            extra_points += frame_buffer_[i]->points.size();
        }
        cloud->points.reserve(cloud->points.size() + extra_points);

        for (size_t i = 0; i < frame_buffer_.size() - 1; ++i)
        {
            cloud->points.insert(
                cloud->points.end(),
                frame_buffer_[i]->points.begin(),
                frame_buffer_[i]->points.end());
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
    }
}

void lidar_cluster::deduplicateDetections(std::vector<ConeDetection>& cones,
                                           pcl::PointCloud<PointType>::Ptr& cones_cloud)
{
    const double radius_sq = config_.dedup.radius * config_.dedup.radius;
    const size_t n = cones.size();
    if (n <= 1) return;

    // Sort indices by confidence descending
    std::vector<size_t> order(n);
    for (size_t i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return cones[a].confidence > cones[b].confidence;
    });

    // Greedy suppression: keep highest-confidence, suppress neighbors within radius
    std::vector<bool> suppressed(n, false);
    for (size_t i = 0; i < n; ++i) {
        size_t idx = order[i];
        if (suppressed[idx]) continue;
        for (size_t j = i + 1; j < n; ++j) {
            size_t jdx = order[j];
            if (suppressed[jdx]) continue;
            double dx = cones[idx].centroid.x - cones[jdx].centroid.x;
            double dy = cones[idx].centroid.y - cones[jdx].centroid.y;
            if (dx * dx + dy * dy < radius_sq) {
                suppressed[jdx] = true;
            }
        }
    }

    // Rebuild cones and cones_cloud
    std::vector<ConeDetection> kept;
    kept.reserve(n);
    pcl::PointCloud<PointType>::Ptr new_cloud(new pcl::PointCloud<PointType>);
    for (size_t i = 0; i < n; ++i) {
        if (suppressed[i]) continue;
        kept.push_back(std::move(cones[i]));
        if (kept.back().cluster) {
            new_cloud->points.insert(new_cloud->points.end(),
                                     kept.back().cluster->points.begin(),
                                     kept.back().cluster->points.end());
        }
    }
    new_cloud->width = new_cloud->points.size();
    new_cloud->height = 1;
    new_cloud->is_dense = true;
    cones = std::move(kept);
    cones_cloud = new_cloud;
}