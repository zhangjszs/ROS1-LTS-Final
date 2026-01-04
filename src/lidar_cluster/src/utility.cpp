#include <utility.h>

struct comp
{
    bool operator()(PointType a, PointType b)
    {
        if (a.x < b.x)
            return true;
        else if (a.x > b.x)
            return false;
        else if (a.y < b.y)
            return true;
        else if (a.y > b.y)
            return false;
        else if (a.z > b.z)
            return true;
        else if (a.z > b.z)
            return false;
    }
};
set<PointType, comp> orderCloud;

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
        //    if ( tmp_euc<=15 && in->points[i].x>0 && tmp_euc>=0.5)
        //    //in->points[i].x>0
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

void lidar_cluster::init()
{
    // m<< 0.976219,-0.216786,0,   1.19277,
    //     0.216786, 0.976219,0,   -0.328103,
    //     0,        0,       1,   0,
    //     0,        0,       0,   1;
    // m<< 0.976219,-0.216786,0,   0,
    //         0.216786, 0.976219,0,   0,
    //         0,        0,       1,   0,
    //         0,        0,       0,   1;
    if (!private_nh_.param<std::string>("input_topic", input_topic,
                                        "/velodyne_points"))
    {
        ROS_WARN_STREAM(
            "Did not load input_topic topic name. Standard value is: "
            << input_topic);
    }
    if (!private_nh_.param<std::string>("no_ground_point_topic",
                                        no_ground_topic, "/points_no_ground"))
    {
        ROS_WARN_STREAM(
            "Did not load points_no_ground topic name. Standard value is: "
            << no_ground_topic);
    }
    if (!private_nh_.param<std::string>("ground_point_topic", ground_topic,
                                        "/points_ground"))
    {
        ROS_WARN_STREAM(
            "Did not load points_ground topic name. Standard value is: "
            << ground_topic);
    }
    if (!private_nh_.param<std::string>("all_points_topic", all_points_topic,
                                        "/all_points"))
    {
        ROS_WARN_STREAM(
            "Did not load points_ground topic name. Standard value is: "
            << all_points_topic);
    }
    if (!private_nh_.param<double>("clip_height", clip_height_, 2.0))
    {
        ROS_WARN_STREAM(
            "Did not load clip_height. Standard value is: " << clip_height_);
    }
    if (!private_nh_.param<double>("sensor_height", sensor_height_, 0.135))
    {
        ROS_WARN_STREAM("Did not load sensor_height. Standard value is: "
                        << sensor_height_);
    }
    if (!private_nh_.param<double>("min_distance", min_distance_, 0.1))
    {
        ROS_WARN_STREAM(
            "Did not load min_distance. Standard value is: " << min_distance_);
    }
    if (!private_nh_.param<double>("max_distance", max_distance_, 100.0))
    {
        ROS_WARN_STREAM(
            "Did not load sensor_height. Standard value is: " << max_distance_);
    }
    if (!private_nh_.param<double>("sensor_height", sensor_height_, 100.0))
    {
        ROS_WARN_STREAM("Did not load sensor_height. Standard value is: "
                        << sensor_height_);
    }
    if (!private_nh_.param<int>("sensor_model", sensor_model_, 16))
    {
        ROS_WARN_STREAM(
            "Did not load sensor_model. Standard value is: " << sensor_model_);
    }
    if (!private_nh_.param<int>("num_iter", num_iter_, 3))
    {
        ROS_WARN_STREAM(
            "Did not load num_iter. Standard value is: " << num_iter_);
    }
    if (!private_nh_.param<int>("num_lpr", num_lpr_, 5))
    {
        ROS_WARN_STREAM(
            "Did not load num_lpr. Standard value is: " << num_lpr_);
    }
    if (!private_nh_.param<double>("th_seeds", th_seeds_, 0.03))
    {
        ROS_WARN_STREAM(
            "Did not load th_seed. Standard value is: " << th_seeds_);
    }
    if (!private_nh_.param<double>("th_dist", th_dist_, 0.03))
    {
        ROS_WARN_STREAM(
            "Did not load th_seed. Standard value is: " << th_seeds_);
    }
    if (!private_nh_.param<int>("road_type", road_type_, 2))
    {
        ROS_WARN_STREAM(
            "Did not load road_type. Standard value is: " << road_type_);
    }
    if (!private_nh_.param<double>("z_up", z_up_, 0.7))
    {
        ROS_WARN_STREAM("Did not load z_up. Standard value is: " << z_up_);
    }
    if (!private_nh_.param<double>("z_down", z_down_, -1.0))
    {
        ROS_WARN_STREAM(
            "Did not load road_type. Standard value is: " << z_down_);
    }
    if (!private_nh_.param<int>("vis", vis_, 0))
    {
        ROS_WARN_STREAM("Did not load vis. Standard value is: " << vis_);
    }
    if (!private_nh_.param<std::string>("str_range", str_range_, "15,30,45,60"))
    {
        ROS_WARN_STREAM(
            "Did not load str_range. Standard value is: " << str_range_);
    }
    if (!private_nh_.param<std::string>("str_seg_distance", str_seg_distance_,
                                        "0.5,1.1,1.6,2.1,2.6"))
    {
        ROS_WARN_STREAM("Did not load str_seg_distance. Standard value is: "
                        << str_seg_distance_);
    }
    // above that is original

    if (!private_nh_.param<double>("min_height", min_height_, -1))
    {
        ROS_WARN("Did not load min_height.");
    }
    if (!private_nh_.param<double>("max_height", max_height_, -1))
    {
        ROS_WARN("Did not load max_height.");
    }
    if (!private_nh_.param<double>("min_area", min_area_, -1))
    {
        ROS_WARN("Did not load min_area.");
    }
    if (!private_nh_.param<double>("max_area", max_area_, -1))
    {
        ROS_WARN("Did not load max_area.");
    }

    // 加载x方向最大加速度
    if (!private_nh_.param<double>("accel_x_max", accel_x_max_, 0))
    {
        ROS_WARN("Did not load accel_x_max.");
    }

    // 加载x方向最小加速度
    if (!private_nh_.param<double>("accel_x_min", accel_x_min_, 0))
    {
        ROS_WARN("Did not load accel_x_min.");
    }

    // 加载y方向最大加速度
    if (!private_nh_.param<double>("accel_y_max", accel_y_max_, 0))
    {
        ROS_WARN("Did not load accel_y_max.");
    }

    // 加载y方向最小加速度
    if (!private_nh_.param<double>("accel_y_min", accel_y_min_, 0))
    {
        ROS_WARN("Did not load accel_y_min.");
    }

    // track_x_max
    if (!private_nh_.param<double>("track_x_max", track_x_max_, 0))
    {
        ROS_WARN("Did not load track_x_max.");
    }

    // track_x_min
    if (!private_nh_.param<double>("track_x_min", track_x_min_, 0))
    {
        ROS_WARN("Did not load track_x_min.");
    }

    // track_y_max
    if (!private_nh_.param<double>("track_y_max", track_y_max_, 0))
    {
        ROS_WARN("Did not load track_y_max.");
    }

    // track_y_min
    if (!private_nh_.param<double>("track_y_min", track_y_min_, 0))
    {
        ROS_WARN("Did not load track_y_min.");
    }

    // skid_x_max
    if (!private_nh_.param<double>("skid_x_max", skid_x_max_, 0))
    {
        ROS_WARN("Did not load skid_x_max.");
    }

    // skid_x_min
    if (!private_nh_.param<double>("skid_x_min", skid_x_min_, 0))
    {
        ROS_WARN("Did not load skid_x_min.");
    }

    // skid_y_max
    if (!private_nh_.param<double>("skid_y_max", skid_y_max_, 0))
    {
        ROS_WARN("Did not load skid_y_max.");
    }

    // skid_y_min
    if (!private_nh_.param<double>("skid_y_min", skid_y_min_, 0))
    {
        ROS_WARN("Did not load skid_y_min.");
    }

    if (!private_nh_.param<double>("max_box_altitude", max_box_altitude_, 0))
    {
        ROS_WARN("Did not load max_box_altitude.");
    }

    // list all parameters at start up
    ROS_INFO("clip_height: %f", clip_height_);
    ROS_INFO("sensor_height: %f", sensor_height_);
    ROS_INFO("min_distance: %f", min_distance_);
    ROS_INFO("max_distance: %f", max_distance_);
    ROS_INFO("sensor_model: %d", sensor_model_);
    ROS_INFO("num_iter: %d", num_iter_);
    ROS_INFO("num_lpr: %d", num_lpr_);
    ROS_INFO("th_seeds: %f", th_seeds_);
    ROS_INFO("th_dist: %f", th_dist_);
    ROS_INFO("road_type: %d", road_type_);
    ROS_INFO("Z-axis clip limit\nUp: %f, Down: %f", z_up_, z_down_);
    ROS_INFO("vis: %d", vis_);

    splitString(str_range_, dis_range);
    splitString(str_seg_distance_, seg_distances);

    getPointClouds = false;
    g_seeds_pc.reset(new pcl::PointCloud<PointType>());
    g_ground_pc.reset(new pcl::PointCloud<PointType>());
    g_not_ground_pc.reset(new pcl::PointCloud<PointType>());
    g_all_pc.reset(new pcl::PointCloud<PointType>());
    current_pc_ptr.reset(new pcl::PointCloud<PointType>());
    cloud_filtered.reset(new pcl::PointCloud<PointType>());
    StatisticalOutlierFilter.reset(new pcl::PointCloud<PointType>());
    skidpad_detection_pc.reset(new pcl::PointCloud<PointType>());
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

    // ROS_INFO("Conf: %f", score);
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

void lidar_cluster::octreeRemovePoints(pcl::PointCloud<PointType>::Ptr &cloud,
                                       pcl::PointCloud<PointType>::Ptr &output)
{
    pcl::octree::OctreePointCloud<PointType> octree(0.1);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud(); // 构建Octree
    vector<int> vec_point_index,
        vec_total_index; //  体素内点的索引，   要删除的点的索引
    vector<pcl::octree::OctreeKey> vec_key;
    for (auto iter = octree.leaf_depth_begin(); iter != octree.leaf_depth_end(); ++iter)
    {
        auto key = iter.getCurrentOctreeKey();
        vec_key.emplace_back(key);
        auto it_key = octree.findLeaf(key.x, key.y, key.z);
        if (it_key != nullptr)
        {
            vec_point_index = iter.getLeafContainer().getPointIndicesVector();
            if (vec_point_index.size() < 20) // 体素内点小于10时删除
            {
                for (size_t i = 0; i < vec_point_index.size(); i++)
                {
                    vec_total_index.push_back(vec_point_index[i]);
                }
            }
        }
    }

    // 使用pcl index 滤波
    pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
    outliners->indices.resize(vec_total_index.size());
    for (size_t i = 0; i < vec_total_index.size(); i++)
    {
        outliners->indices[i] = vec_total_index[i];
    }
    // pcl::ExtractIndices<PointType> cliper;
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(outliners);
    extract.setNegative(false);
    extract.filter(*output);
}

// 直通滤波
void lidar_cluster::PassThrough(
    pcl::PointCloud<PointType>::Ptr &cloud_filtered,
    pcl::PointCloud<PointType>::Ptr &StatisticalOutlierFilter)
{
    // 当线由车后延到车头这个方向，左右是y，前后是x，上下是z，右y是负的，左y是正的
    // 前x为正
    pcl::PassThrough<PointType> pass;
    // ROS_INFO_STREAM("Before PassThrough: " << cloud_filtered->points.size());
    pass.setInputCloud(cloud_filtered);

    if (road_type_ == 1)
    {
        // skidpad
        point_clip(cloud_filtered);
        // pass.setFilterFieldName("x");//x:-1 represent down +1 represent
        // forward pass.setFilterLimits(0.1,20); pass.filter(*cloud_filtered);
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
        ROS_ERROR_STREAM("Received undefined road_type:" << road_type_);
        exit(1);
    }

    pass.setFilterFieldName("z");
    pass.setFilterLimits(
        z_down_,
        z_up_); // 这里的变化是会影响到整体的一个分割效果，z范围越广检测到远处的就越多
    pass.filter(*cloud_filtered);

    //-0.5 0.85 符合3

    // ROS_INFO_STREAM("Before SOR: " << cloud_filtered->points.size());

    // 离群点移除 (Statistical Outlier Removal)
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(0.05f, 0.05f,
                    0.05f); // 设置体素过滤器的体素大为(0.05, 0.05,
                            // 0.05)。体素大小越小，过滤效果越好，但计算量也越大
    sor.filter(*cloud_filtered);

    // pcl::io::savePCDFileASCII("/home/adams/postsynced_msf/pcdtest/filter/cloud_filtered.pcd",
    // *cloud_filtered);

    // auto startTimeSeg = std::chrono::steady_clock::now();
    // octreeRemovePoints(cloud_filtered,StatisticalOutlierFilter);
    // pcl::StatisticalOutlierRemoval<PointType> s;
    // s.setInputCloud(cloud_filtered);
    // s.setMeanK(10);
    // s.setStddevMulThresh(1);
    // s.filter(*StatisticalOutlierFilter);
    // auto endTimeSeg = std::chrono::steady_clock::now();
    // auto elapsedTimeSeg =
    // std::chrono::duration_cast<std::chrono::microseconds>(endTimeSeg -
    // startTimeSeg); std::cout << "inner took " << elapsedTimeSeg.count() << "
    // milliseconds" << std::endl;

    // pcl::toROSMsg(*cloud_filtered, pub_pc);
    // pub_pc.header = in_pc.header;
    // pub_filtered_points_.publish(pub_pc);

    // sof: Statistical Outlier Filter 的缩写,表示统计学离群点滤波
    // std::cout << "before sof pointsize:" << cloud_filtered->points.size() <<
    // std::endl; std::cout << "after sof pointsize:" <<
    // StatisticalOutlierFilter->points.size() << std::endl;

    // pcl::io::savePCDFileASCII("/home/adams/postsynced_msf/pcdtest/filter/StatisticalOutlierFilter.pcd",
    // *StatisticalOutlierFilter); exit(0);
}

// void lidar_cluster::guidedFilter(const pcl::PointCloud<PointType>::Ptr&
// cloud_filtered, pcl::PointCloud<PointType>::Ptr& cloud_filtered_out, unsigned
// int k, float eps){
//     if (!cloud_filtered->points.size())
//         return ;
//     std::cout<<"before sor
//     pointsize:"<<cloud_filtered->points.size()<<std::endl;
//     pcl::KdTreeFLANN<PointType> kdtree;
//     // unsigned int k = 20;//参数1
//     // double epsilon = 0.05;//参数2
//     kdtree.setInputCloud(cloud_filtered);
//     std::cout<<"shared:"<<std::endl;
//     for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
//     {
//         std::vector<int>indices(0,0);
//         indices.reserve(k);
//         std::vector<float>dist(0,0.0);
//         pcl::PointCloud<PointType>::Ptr neigh_points(new
//         pcl::PointCloud<PointType>); dist.reserve(k); if
//         (kdtree.nearestKSearch(cloud_filtered->points[i], k, indices, dist) >
//         0)
//         {
//             pcl::copyPointCloud(*cloud_filtered, indices, *neigh_points);
//             PointType point_mean(0.0, 0.0, 0.0);
//             double neigh_mean_2=0.0;
//             for (auto neigh_point : neigh_points->points)
//             {
//                 point_mean.x += neigh_point.x;
//                 point_mean.y += neigh_point.y;
//                 point_mean.z += neigh_point.z;
//                 neigh_mean_2 += ((neigh_point.x) *
//                 double(neigh_point.x))+(double(neigh_point.y) *
//                 double(neigh_point.y))+(double(neigh_point.z) *
//                 double(neigh_point.z));

//             }
//             point_mean.x /= neigh_points->points.size();
//             point_mean.y /= neigh_points->points.size();
//             point_mean.z /= neigh_points->points.size();
//             neigh_mean_2 /= neigh_points->points.size();

//             double point_mean_2 = (point_mean.x * point_mean.x) +
//             (point_mean.y*point_mean.y) +( point_mean.z * point_mean.z);
//             double a = (neigh_mean_2 - point_mean_2) / (neigh_mean_2 -
//             point_mean_2 + eps); PointType b; b.x = (1.0 - a) * point_mean.x;
//             b.y = (1.0 - a) * point_mean.y;
//             b.z = (1.0 - a) * point_mean.z;

//             PointType smoothed_point(0.0, 0.0, 0.0);
//             smoothed_point.x =a* cloud_filtered->points[i].x + b.x;
//             smoothed_point.y = a * cloud_filtered->points[i].y + b.y;
//             smoothed_point.z = a * cloud_filtered->points[i].z + b.z;
//             cloud_filtered_out->points.push_back(smoothed_point);
//        }
//     }
//     std::cout<<"before sor
//     pointsize:"<<cloud_filtered_out->points.size()<<std::endl; return ;
// }

// void lidar_cluster::EucClusterMethod(const pcl::PointCloud<PointType>::Ptr&
// in_cloud,
//     std::vector<ClusterPtr>& clusters, const double& max_cluster_dis)
// {
//     pcl::search::KdTree<PointType>::Ptr tree(new
//     pcl::search::KdTree<PointType>);

//     pcl::PointCloud<PointType>::Ptr cloud_2d(new pcl::PointCloud<PointType>);

//     pcl::copyPointCloud(*in_cloud, *cloud_2d);

//     for (size_t i = 0; i < cloud_2d->points.size(); i++) {
//         cloud_2d->points[i].z = 0;
//     }

//     tree->setInputCloud(cloud_2d);
//     std::vector<pcl::PointIndices> cluster_indices;

//     pcl::EuclideanClusterExtraction<PointType> euc;
//     // setClusterTolerance(). If you take a very small value, it can happen
//     that
//     // an actual object can be seen as multiple clusters. On the other hand,
//     if
//     // you set the value too high, it could happen, that multiple objects are
//     // seen as one cluster. So our recommendation is to just test and try out
//     // which value suits your dataset.
//     euc.setClusterTolerance(max_cluster_dis);
//     euc.setMinClusterSize(cluster_min_points);
//     euc.setMaxClusterSize(cluster_max_points);
//     euc.setSearchMethod(tree);
//     euc.setInputCloud(cloud_2d);
//     euc.extract(cluster_indices);

//     for (size_t j = 0; j < cluster_indices.size(); j++) {
//         ClusterPtr one_cluster;
//         one_cluster->setCloud(in_cloud, color_table,
//         cluster_indices[j].indices, j); clusters.push_back(one_cluster);
//     }
// }

void lidar_cluster::EucClusterMethod(
    pcl::PointCloud<PointType>::Ptr inputcloud,
    std::vector<pcl::PointIndices> &cluster_indices,
    const double &max_cluster_dis)
{
    if (inputcloud->points.size() == 0)
    {
        // ROS_WARN("EucClusterMethod recived empty input cloud! Exiting...");
        return;
    }

    // auto startTime = std::chrono::steady_clock::now();
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
    // auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime =
    // std::chrono::duration_cast<std::chrono::milliseconds>(endTime -
    // startTime); std::cout << "EucCluster took " << elapsedTime.count() << "
    // milliseconds" << std::endl;
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

    float interval = 0;
    for (size_t i = 0; i < inputcloud->points.size(); i++)
    {
        PointType p;
        p.x = inputcloud->points[i].x;
        p.y = inputcloud->points[i].y;
        p.z = inputcloud->points[i].z;
        p.intensity = inputcloud->points[i].intensity;
        float origin_dis = sqrt(pow(p.x, 2) + pow(p.y, 2)); // 点到原点距离
        // float origin_dis = p.x;
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

void lidar_cluster::clusterMethod32()
{
    if (vis_)
    {
        vis_init_status = vis_init();
    }

    common_msgs::Cone position;

    // position.points.clear();
    // position.pc.clear();
    // position.maxPoints.clear();
    // position.minPoints.clear();

    skidpad_detection_pc->points.clear();

    std::vector<std::vector<pcl::PointIndices>>
        cluster_indices; // 用于存储点云聚类的索引信息
    // split_points_MultiEuc(g_not_ground_pc);

    std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array(5);
    for (size_t i = 0; i < cloud_segments_array.size(); i++)
    {
        pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>);
        cloud_segments_array[i] = tmp;
    }

    EuclideanAdaptiveClusterMethod(g_not_ground_pc, cluster_indices,
                                   cloud_segments_array);

    // EucClusterMethod(g_not_ground_pc, cluster_indices, 0.3);
    int j = 0;
    int clusterId = 0;
    pcl::PointCloud<PointType>::Ptr final_cluster(
        new pcl::PointCloud<PointType>);

    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        // loop that runs according to cluster size

        for (std::vector<pcl::PointIndices>::const_iterator it =
                 cluster_indices[i].begin();
             it != cluster_indices[i].end(); ++it)
        {
            // loop for cluster block, it equals each cluster block index

            // init cluster var
            pcl::PointCloud<PointType>::Ptr cloud_cluster(
                new pcl::PointCloud<PointType>);

            std::vector<int> intensity;
            Eigen::Vector4f centroid;

            intensity_count = 0;
            intensity_min = 99999;
            intensity_max = -99999;

            // add point cloud data to cluster
            std::vector<int> indices = it->indices;
            for (int idx : indices)
            {
                // another solve; equals to ln.563
                cloud_cluster->points.push_back(
                    cloud_segments_array[i]->points[idx]);
            }

            // for (std::vector<int>::const_iterator pit = it->indices.begin();
            // pit != it->indices.end(); pit++)
            // {

            //     // loop for every point in cluster block

            //     // when comment these code below, the output is 6 cones
            //     (left/right 3 each)
            //     // when using original code , the output was the same

            //     // this loop just for building data from single points?
            //     // functionality has been replaced by ln.535

            //     //
            //     intensity.push_back(g_not_ground_pc->points[*pit].intensity);

            //     //
            //     cloud_cluster->points.push_back(g_not_ground_pc->points[*pit]);

            //     // 在每次循环中添加每个聚类块的所有点
            //     //
            //     cloud_cluster->points.push_back(cloud_segments_array[i]->points[*pit]);
            //     // ORIGINAL CODE

            //     // intensity += g_not_ground_pc->points[*pit].intensity;
            // }

            // ---------------------------------- intensity number
            // analysis--------------------------------------- float sum =
            // std::accumulate(intensity.begin(),intensity.end(),0); float accum
            // = 0.0; intensity_mean = sum / (intensity.size());
            // std::for_each(intensity.begin(),intensity.end(),[&](const double
            // d){
            //     accum += (d-intensity_mean)*(d-intensity_mean);
            // });
            // float stdev = sqrt(accum/(intensity.size()-1));
            // auto itMax = std::max_element(intensity.begin(),intensity.end());
            // auto itMin = std::min_element(intensity.begin(),intensity.end());
            // ---------------------------------- intensity number
            // analysis---------------------------------------

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            PointType _min;
            PointType _max;
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            pcl::getMinMax3D(*cloud_cluster, _min, _max);

            double confidence = getConfidence(_max, _min, centroid);

            // start cone judgement
            if (confidence > 0.5)
            // TODO make these parameters can be configured in yaml files
            {
                // identified as "CONE"
                position.confidence.push_back(confidence);

                *final_cluster = *final_cluster + *cloud_cluster;
                pcl::toROSMsg(*cloud_cluster, cluster_for_publish);

                cluster_for_publish.header = in_pc.header;
                position.pc.push_back(cluster_for_publish);

                geometry_msgs::Point32 tmp, tmp1, tmp2;
                tmp.x = centroid[0];
                tmp.y = centroid[1];
                tmp.z = centroid[2];

                // distance is based on vehicle location
                euc = float(sqrt(centroid[0] * centroid[0] +
                                 centroid[1] * centroid[1]));

                position.points.push_back(
                    tmp); // cone position of centroid published
                position.obj_dist.push_back(euc);
                tmp1.x = _max.x;
                tmp1.y = _max.y;
                tmp1.z = _max.z;
                position.maxPoints.push_back(tmp1);
                tmp2.x = _min.x;
                tmp2.y = _min.y;
                tmp2.z = _min.z;
                position.minPoints.push_back(tmp2);
                // euc = intensity;
                if (vis_ && vis_init_status)
                {
                    vis_for_marker(_max, _min, euc, 0, 0, 0, true);
                    // vis_for_marker(max,min,euc,*itMax,*itMin,stdev);
                }
            }
            else
            {
                // identified as "NOT CONES"
                if (vis_ && vis_init_status)
                {
                    vis_for_marker(_max, _min, euc, 0, 0, 0, false);
                    // vis_for_marker(max,min,euc,*itMax,*itMin,stdev);
                }
                continue;
            }

            ++clusterId;
            ++j;

            PointType tmp_pXYZ;
            tmp_pXYZ.x = centroid[0];
            tmp_pXYZ.y = centroid[1];
            tmp_pXYZ.z = centroid[2];

            skidpad_detection_pc->points.push_back(tmp_pXYZ);
            // orderCloud.insert(tmp_pc);

            // out_pc.points.push_back(tmp);//only centroid points
            // position.points.push_back(tmp);//cone position of centroid
            // published delete cloud_cluster;
        }
    }

    // sensor_msgs::PointCloud2 pub_pc;
    // out = final_cluster;
    pcl::toROSMsg(*final_cluster, pub_pc);
    pub_pc.header = in_pc.header;
    pub_filtered_points__.publish(pub_pc);

    position.pc_whole = pub_pc;
    position.pc_whole.header = in_pc.header;
    markerPub.publish(marker_array);
    markerPubAll.publish(marker_array_all);

    pcl::toROSMsg(*skidpad_detection_pc, pub_pc);
    pub_pc.header = in_pc.header;
    skidpad_detection.publish(pub_pc);
    position.header = in_pc.header;

    cone_position.publish(position);

    // ROS_INFO_STREAM("PUBLISH HEADERS" << in_pc.header.stamp);
}

void lidar_cluster::clusterMethod16()
{
    if (vis_)
    {
        bool vis_init_status = vis_init();
    }
    orderCloud.clear();
    common_msgs::Cone position;
    // position.points.clear();
    // position.pc.clear();
    skidpad_detection_pc->points.clear();
    std::vector<pcl::PointIndices> cluster_indices;

    EucClusterMethod(g_not_ground_pc, cluster_indices, 0.3);
    int j = 0;
    int clusterId = 0;
    pcl::PointCloud<PointType>::Ptr final_cluster(
        new pcl::PointCloud<PointType>);
    // for(pcl::PointIndices getIndices: cluster_indices){}
    for (std::vector<pcl::PointIndices>::const_iterator it =
             cluster_indices.begin();
         it != cluster_indices.end(); ++it)
    {
        // for(const auto *it : cluster_indices){
        pcl::PointCloud<PointType>::Ptr cloud_cluster(
            new pcl::PointCloud<PointType>);
        // for(int index : getIndices.indices)
        //  intensity = 0.0;
        intensity_count = 0;
        for (std::vector<int>::const_iterator pit = it->indices.begin();
             pit != it->indices.end(); pit++)
        {
            // for(const auto *pit:it->indices){
            // cloudCluster->point.push_bach(cloud->points[index]);
            // intensity_count++;
            cloud_cluster->points.push_back(
                g_not_ground_pc
                    ->points[*pit]); //*
                                     // cloud_cluster->points.push_back
                                     // (cloud_segments_array[i]->points[*pit]);
                                     // //* intensity +=
                                     // g_not_ground_pc->points[*pit].intensity;
        }
        // intensity = intensity / cloud_cluster->points.size();
        // cloud_cluster->width = cloud_cluster->points.size ();
        // cloud_cluster->height = 1;
        // cloud_cluster->is_dense = true;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);

        PointType min;
        PointType max;
        pcl::getMinMax3D(*cloud_cluster, min, max);
        double xx = (double)std::fabs(max.x - min.x);
        double yy = (double)std::fabs(max.y - min.y);
        double zz = (double)std::fabs(max.z - min.z);
        // if( (xx < 0.4 && yy < 0.4 && zz < 1 && (zz > 0.1 || centroid[2]>0.05)
        // && centroid[2]< 0.7 && centroid[2] > -0.1) )//x
        if ((xx < 0.4 && yy < 0.4 && zz < 0.5)) // x
        // if( (xx < 0.4 && yy < 0.4 && zz < 1 && centroid[2]< 0.6 &&
        // centroid[2] > -0.1) )//x
        {
            *final_cluster = *final_cluster + *cloud_cluster;
            pcl::toROSMsg(*cloud_cluster, cluster_for_publish);

            cluster_for_publish.header = in_pc.header;
            position.pc.push_back(cluster_for_publish);

            geometry_msgs::Point32 tmp, tmp1, tmp2;
            tmp.x = centroid[0];
            tmp.y = centroid[1];
            tmp.z = centroid[2];
            euc = float(
                sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]));
            position.points.push_back(
                tmp); // cone position of centroid published
            position.obj_dist.push_back(euc);
            tmp1.x = max.x;
            tmp1.y = max.y;
            tmp1.z = max.z;
            position.maxPoints.push_back(tmp1);
            tmp2.x = min.x;
            tmp2.y = min.y;
            tmp2.z = min.z;
            position.minPoints.push_back(tmp2);
            // euc = intensity;
            if (vis_ && vis_init_status)
            {
                vis_for_marker(max, min, euc, 0, 0, 0, true);
                // vis_for_marker(max,min,euc,*itMax,*itMin,stdev);
            }
        }
        else
            continue;
        // std::cout<<clusterId<<"Cluster_Centroid"<<"("
        // << centroid[0] << ","
        // << centroid[1] << ","
        // << centroid[2] << ")."<<std::endl;
        ++clusterId;
        ++j;
        // geometry_msgs::Point32 tmp;
        // PointType tmp_pc;
        // tmp.x=centroid[0];
        // tmp.y=centroid[1];
        // tmp.z=centroid[2];
        PointType tmp_pXYZ;
        tmp_pXYZ.x = centroid[0];
        tmp_pXYZ.y = centroid[1];
        tmp_pXYZ.z = centroid[2];
        // tmp_pc.x=centroid[0];
        // tmp_pc.y=centroid[1];
        // tmp_pc.z=centroid[2];
        skidpad_detection_pc->points.push_back(tmp_pXYZ);
        // orderCloud.insert(tmp_pc);

        // out_pc.points.push_back(tmp);//only centroid points
        // position.points.push_back(tmp);//cone position of centroid published
        // delete cloud_cluster;
    }

    // gettimeofday(&end,NULL);

    markerPub.publish(marker_array);
    // sensor_msgs::PointCloud2 pub_pc;
    // out = final_cluster;
    pcl::toROSMsg(*final_cluster, pub_pc);
    pub_pc.header = in_pc.header;
    pub_filtered_points__.publish(pub_pc);

    pcl::toROSMsg(*skidpad_detection_pc, pub_pc);
    pub_pc.header = in_pc.header;
    skidpad_detection.publish(pub_pc);

    cone_position.publish(position);
}