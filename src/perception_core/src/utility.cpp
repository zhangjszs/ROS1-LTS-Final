#include <perception_core/lidar_cluster_core.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <iostream>
#include <sstream>
#include <boost/make_shared.hpp>

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

}  // namespace

float euc_distance(PointType a) { return float(sqrt(a.x * a.x + a.y * a.y)); }

void point_clip(const pcl::PointCloud<PointType>::Ptr in)
{
    pcl::ExtractIndices<PointType> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    float tmp_euc;
    // P2: 移除不安全的 #pragma omp for（存在数据竞争，indices.indices 不是线程安全的）
    for (size_t i = 0; i < in->points.size(); i++)
    {
        tmp_euc = euc_distance(in->points[i]);
        if (tmp_euc <= 15 && in->points[i].x > 0 &&
            tmp_euc >= 1) // in->points[i].x>0
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(
        false); // ture to remove the indices false to retain the indices
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
    road_type_ = config.road_type;
    ground_method_ = config.ground_method;
    roi_ = config.roi;
    filters_ = config.filters;
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
    confidence_scorer_.setConfig(scorer_config);

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

    dis_range.clear();
    seg_distances.clear();
    splitString(str_range_, dis_range);
    splitString(str_seg_distance_, seg_distances);
    
    // P2: 参数校验 - 确保 dis_range 和 seg_distances 有效
    if (dis_range.size() < 4) {
        std::cerr << "[Warning] str_range has less than 4 values, using defaults: 15,30,45,60" << std::endl;
        dis_range = {15.0, 30.0, 45.0, 60.0};
    }
    if (seg_distances.size() < 5) {
        std::cerr << "[Warning] str_seg_distance has less than 5 values, using defaults: 0.15,0.3,0.5,0.6,0.6" << std::endl;
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
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud_filtered);

    // 优化顺序：Z轴裁剪 -> 体素降采样 -> ROI裁剪 -> SOR
    // 1. Z轴裁剪（最快，先去除明显非目标点）
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_down_, z_up_);
    pass.filter(*cloud_filtered);

    // 2. 体素降采样（减少后续处理点数）
    if (filters_.adaptive_voxel.enable)
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

    // 3. ROI裁剪（X, Y轴）
    pass.setInputCloud(cloud_filtered);
    if (road_type_ == 1)
    {
        // skidpad
        if (roi_use_point_clip_)
        {
            point_clip(cloud_filtered);
        }
        else
        {
            pass.setFilterFieldName("x");
            pass.setFilterLimits(skid_x_min_, skid_x_max_);
            pass.filter(*cloud_filtered);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(skid_y_min_, skid_y_max_);
            pass.filter(*cloud_filtered);
        }
    }
    else if (road_type_ == 2)
    {
        // acclerate
        pass.setFilterFieldName("x");
        pass.setFilterLimits(accel_x_min_, accel_x_max_);
        pass.filter(*cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(accel_y_min_, accel_y_max_);
        pass.filter(*cloud_filtered);
    }
    else if (road_type_ == 3)
    {
        // track
        pass.setFilterFieldName("x");
        pass.setFilterLimits(track_x_min_, track_x_max_);
        pass.filter(*cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(track_y_min_, track_y_max_);
        pass.filter(*cloud_filtered);
    }
    else
    {
        std::cerr << "[Warning] Received undefined road_type: " << road_type_
                  << ", falling back to track mode (road_type=3)" << std::endl;
        road_type_ = 3;
        pass.setFilterFieldName("x");
        pass.setFilterLimits(track_x_min_, track_x_max_);
        pass.filter(*cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(track_y_min_, track_y_max_);
        pass.filter(*cloud_filtered);
    }

    // 4. SOR离群点移除（在降采样后的点云上执行，更高效）
    if (filters_.sor.enable)
    {
        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setMeanK(filters_.sor.mean_k);
        sor.setStddevMulThresh(filters_.sor.stddev_mul);
        sor.filter(*cloud_filtered);
    }
}

void lidar_cluster::EucClusterMethod(
    pcl::PointCloud<PointType>::Ptr inputcloud,
    std::vector<pcl::PointIndices> &cluster_indices,
    const double &max_cluster_dis)
{
    if (inputcloud->points.size() == 0)
    {
        return;
    }
    pcl::search::KdTree<PointType>::Ptr tree(
        new pcl::search::KdTree<PointType>);
    tree->setInputCloud(inputcloud);
    pcl::EuclideanClusterExtraction<PointType> ec; // 聚类对象
    ec.setClusterTolerance(max_cluster_dis); // 设置邻近搜索的搜索半径为3cm
    ec.setMinClusterSize(2); // 设置一个聚类需要的最少点云数目为100  2
    ec.setMaxClusterSize(50);     // 最多点云数目为20000        50
    ec.setSearchMethod(tree);     // 设置点云的搜索机制
    ec.setInputCloud(inputcloud); // 设置原始点云
    ec.extract(cluster_indices);  // 从点云中提取聚类
}

void lidar_cluster::EuclideanAdaptiveClusterMethod(
    pcl::PointCloud<PointType>::Ptr inputcloud,
    std::vector<std::vector<pcl::PointIndices>> &cluster_indices,
    std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array)
{
    // 根据距离不同，设置不同的聚类阈值
    // 0 => 0-15m d=0.5
    // 1 => 15-30 d=1
    // 2 => 30-45 d=1.6
    // 3 => 45-60 d=2.1
    // 4 => >60   d=2.6

    for (size_t i = 0; i < inputcloud->points.size(); i++)
    {
        PointType p;
        p.x = inputcloud->points[i].x;
        p.y = inputcloud->points[i].y;
        p.z = inputcloud->points[i].z;
        p.intensity = inputcloud->points[i].intensity;
        float origin_dis = sqrt(pow(p.x, 2) + pow(p.y, 2)); // 点到原点距离
        if (origin_dis < dis_range[0])
        {
            cloud_segments_array[0]->points.push_back(p);
        }
        else if (origin_dis < dis_range[1])
        {
            cloud_segments_array[1]->points.push_back(p);
        }
        else if (origin_dis < dis_range[2])
        {
            cloud_segments_array[2]->points.push_back(p);
        }
        else if (origin_dis < dis_range[3])
        {
            cloud_segments_array[3]->points.push_back(p);
        }
        else
        {
            cloud_segments_array[4]->points.push_back(p);
        }
    }
    std::vector<pcl::PointIndices> v;
    for (size_t i = 0; i < cloud_segments_array.size(); i++)
    {
        v.clear();
        EucClusterMethod(cloud_segments_array[i], v,
                         seg_distances[i]); // 自适应分割聚类
        cluster_indices.push_back(v);
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
    output->cones_cloud.reset(new pcl::PointCloud<PointType>);

    std::vector<std::vector<pcl::PointIndices>> cluster_indices;

    std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array(5);
    for (size_t i = 0; i < cloud_segments_array.size(); i++)
    {
        pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>);
        cloud_segments_array[i] = tmp;
    }

    EuclideanAdaptiveClusterMethod(g_not_ground_pc, cluster_indices,
                                   cloud_segments_array);
    size_t total_clusters = 0;
    for (const auto &segment : cluster_indices)
    {
        total_clusters += segment.size();
    }

    pcl::PointCloud<PointType>::Ptr final_cluster(
        new pcl::PointCloud<PointType>);

    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        for (std::vector<pcl::PointIndices>::const_iterator it =
                 cluster_indices[i].begin();
             it != cluster_indices[i].end(); ++it)
        {
            pcl::PointCloud<PointType>::Ptr cloud_cluster(
                new pcl::PointCloud<PointType>);

            std::vector<int> indices = it->indices;
            for (int idx : indices)
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

            double confidence = getConfidenceWithFitting(cloud_cluster);
            bool is_cone = (confidence > 0.3);  // 使用置信度阈值过滤

            float euc = float(sqrt(centroid[0] * centroid[0] +
                                   centroid[1] * centroid[1]));

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
                output->cones.push_back(det);
                *final_cluster = *final_cluster + *cloud_cluster;
            }
            else if (vis_)
            {
                output->non_cones.push_back(det);
            }
        }
    }

    output->cones_cloud = final_cluster;
    output->cones_cloud->width = output->cones_cloud->points.size();
    output->cones_cloud->height = 1;
    output->cones_cloud->is_dense = g_not_ground_pc->is_dense;
    last_cluster_count_ = total_clusters;
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
    output->cones_cloud.reset(new pcl::PointCloud<PointType>);

    std::vector<pcl::PointIndices> cluster_indices;

    EucClusterMethod(g_not_ground_pc, cluster_indices, 0.3);
    size_t total_clusters = cluster_indices.size();

    pcl::PointCloud<PointType>::Ptr final_cluster(
        new pcl::PointCloud<PointType>);

    for (std::vector<pcl::PointIndices>::const_iterator it =
             cluster_indices.begin();
         it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<PointType>::Ptr cloud_cluster(
            new pcl::PointCloud<PointType>);

        for (std::vector<int>::const_iterator pit = it->indices.begin();
             pit != it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(
                g_not_ground_pc->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = g_not_ground_pc->is_dense;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);

        PointType min;
        PointType max;
        pcl::getMinMax3D(*cloud_cluster, min, max);
        double xx = (double)std::fabs(max.x - min.x);
        double yy = (double)std::fabs(max.y - min.y);
        double zz = (double)std::fabs(max.z - min.z);

        bool is_cone = (xx < 0.4 && yy < 0.4 && zz < 0.5);
        float euc = float(sqrt(centroid[0] * centroid[0] +
                               centroid[1] * centroid[1]));

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
            output->cones.push_back(det);
            *final_cluster = *final_cluster + *cloud_cluster;
        }
        else if (vis_)
        {
            output->non_cones.push_back(det);
        }
    }

    output->cones_cloud = final_cluster;
    output->cones_cloud->width = output->cones_cloud->points.size();
    output->cones_cloud->height = 1;
    output->cones_cloud->is_dense = g_not_ground_pc->is_dense;
    last_cluster_count_ = total_clusters;
}
