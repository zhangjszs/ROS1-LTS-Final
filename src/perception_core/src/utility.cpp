#include <perception_core/lidar_cluster_core.hpp>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <boost/make_shared.hpp>

using std::string;
using std::vector;
using std::stod;

float euc_distance(PointType a) { return float(sqrt(a.x * a.x + a.y * a.y)); }

void point_clip(const pcl::PointCloud<PointType>::Ptr in)
{
    pcl::ExtractIndices<PointType> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    float tmp_euc;
#pragma omp for
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
    num_iter_ = config.num_iter;
    num_lpr_ = config.num_lpr;
    th_seeds_ = config.th_seeds;
    th_dist_ = config.th_dist;
    road_type_ = config.road_type;
    z_up_ = config.z_up;
    z_down_ = config.z_down;
    vis_ = config.vis;
    str_range_ = config.str_range;
    str_seg_distance_ = config.str_seg_distance;

    min_height_ = config.min_height;
    max_height_ = config.max_height;
    min_area_ = config.min_area;
    max_area_ = config.max_area;
    max_box_altitude_ = config.max_box_altitude;

    accel_x_max_ = config.accel_x_max;
    accel_x_min_ = config.accel_x_min;
    accel_y_max_ = config.accel_y_max;
    accel_y_min_ = config.accel_y_min;

    track_x_max_ = config.track_x_max;
    track_x_min_ = config.track_x_min;
    track_y_max_ = config.track_y_max;
    track_y_min_ = config.track_y_min;

    skid_x_max_ = config.skid_x_max;
    skid_x_min_ = config.skid_x_min;
    skid_y_max_ = config.skid_y_max;
    skid_y_min_ = config.skid_y_min;

    dis_range.clear();
    seg_distances.clear();
    splitString(str_range_, dis_range);
    splitString(str_seg_distance_, seg_distances);
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
    // 当线由车后延到车头这个方向，左右是y，前后是x，上下是z，右y是负的，左y是正的
    // 前x为正
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud_filtered);

    if (road_type_ == 1)
    {
        // skidpad
        point_clip(cloud_filtered);
    }
    else if (road_type_ == 2)
    {
        // acclerate
        pass.setFilterFieldName("x");
        pass.setFilterLimits(accel_x_min_, accel_x_max_); // 出图用的是5
        pass.filter(*cloud_filtered);
        // 此处必须要在定义 Filter 属性之后应用，否则配置将不会生效

        pass.setFilterFieldName("y"); // y:-1 represent right,+1 represent left
        pass.setFilterLimits(
            accel_y_min_,
            accel_y_max_); // 使用 2 来保证兼容测试情况锥桶摆放不标准的情况
        pass.filter(*cloud_filtered);
    }
    else if (road_type_ == 3)
    {
        // track
        pass.setFilterFieldName("x");
        pass.setFilterLimits(track_x_min_, track_x_max_);
        pass.filter(*cloud_filtered);
        pass.setFilterFieldName("y"); // y:-1 represent right,+1 represent left
        pass.setFilterLimits(track_y_min_, track_y_max_);
        pass.filter(*cloud_filtered);
    }
    else
    {
        std::cerr << "Received undefined road_type: " << road_type_ << std::endl;
        std::exit(1);
    }

    pass.setFilterFieldName("z");
    pass.setFilterLimits(
        z_down_,
        z_up_); // 这里的变化是会影响到整体的一个分割效果，z范围越广检测到远处的就越多
    pass.filter(*cloud_filtered);

    //-0.5 0.85 符合3

    // 离群点移除 (Statistical Outlier Removal)
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(0.05f, 0.05f,
                    0.05f); // 设置体素过滤器的体素大为(0.05, 0.05,
                            // 0.05)。体素大小越小，过滤效果越好，但计算量也越大
    sor.filter(*cloud_filtered);
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

            double confidence = getConfidence(_max, _min, centroid);
            bool is_cone = confidence > 0.5;

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
