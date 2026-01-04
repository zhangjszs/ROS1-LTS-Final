#include <utility.h>

bool lidar_cluster::vis_init()
{
    marker_array.markers.clear();
    marker_array_all.markers.clear();
    marker_id = 0;
    euc_marker.header.frame_id = "velodyne";
    euc_marker.header.stamp = ros::Time::now();
    euc_marker.ns = "euc";
    euc_marker.color.r = 1.0f;
    euc_marker.color.g = 0.0f;
    euc_marker.color.b = 0.0f;
    euc_marker.color.a = 1.0f;
    euc_marker.lifetime = ros::Duration(0.1);
    euc_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    euc_marker.action = visualization_msgs::Marker::ADD;

    bbox_marker.header.frame_id = "velodyne";
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.ns = "lines";
    bbox_marker.color.r = 1.0f;
    bbox_marker.color.g = 1.0f;
    bbox_marker.color.b = 1.0f;
    bbox_marker.color.a = 1.0f;
    bbox_marker.lifetime = ros::Duration(0.1);
    // bbox_marker.frame_locked = true;
    bbox_marker.scale.x = 0.01;
    // bbox_marker.type = visualization_msgs::Marker::CUBE;
    // bbox_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    bbox_marker.type = visualization_msgs::Marker::LINE_LIST;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    return true;
}

void lidar_cluster::vis_for_marker(const PointType max, const PointType min, float euc, float intensity_max, float intensity_min, float intensity_mean, bool type)
{
    // bbox_marker.clear();
    // visualization_msgs::Marker bbox_marker;
    // bbox_marker.header.frame_id = "velodyne";
    // bbox_marker.header.stamp = ros::Time::now();
    // bbox_marker.ns = "lines";
    // bbox_marker.color.r = 1.0f;
    // bbox_marker.color.g = 1.0f;
    // bbox_marker.color.b = 1.0f;
    // bbox_marker.color.a = 1.0f;
    // bbox_marker.lifetime = ros::Duration(0.1);
    // bbox_marker.scale.x = 0.01;
    // bbox_marker.type = visualization_msgs::Marker::LINE_LIST;
    // bbox_marker.action = visualization_msgs::Marker::ADD;
    // euc_marker.clear();
    // std::cout<<"in???"<<std::endl;
    bbox_marker.points.clear();
    bbox_marker.id = marker_id;
    euc_marker.id = marker_id;

    // std::string text_intensity;
    // intensity_max_marker.id = intensity_min_marker.id = intensity_mean_marker.id = marker_id;
    // intensity_max_marker.pose.position.x = (max.x + min.x) / 2;
    // intensity_max_marker.pose.position.y = (max.y + min.y) / 2;
    // intensity_max_marker.pose.position.z = (max.z + min.z) / 2+1;
    // text_intensity = "(" + to_string(intensity_min) + "," + to_string(intensity_max) + "," + to_string(intensity_mean) + ")";
    // intensity_max_marker.text = text_intensity;
    // intensity_max_marker.scale.z = 0.2;
    // ROS_INFO_STREAM("intensity_information: "<< text_intensity);

    euc_marker.pose.position.x = (max.x + min.x) / 2;
    euc_marker.pose.position.y = (max.y + min.y) / 2;
    euc_marker.pose.position.z = (max.z + min.z) / 2 + 0.5;
    // euc = float(sqrt(centroid[0]*centroid[0] +centroid[1]*centroid[1]));

    std::string text;
    text = to_string(euc);

    // geometry_msgs::Point p;
    euc_marker.text = text;
    euc_marker.scale.z = 1;

    p.x = min.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 左下

    p.x = min.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 右下

    p.x = max.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 左上

    p.x = max.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 右上

    p.x = min.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 左下

    p.x = max.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 左上

    p.x = min.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 右下

    p.x = max.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 底右上

    p.x = min.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 左下

    p.x = min.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 右下

    p.x = max.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 左上

    p.x = max.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 右上

    p.x = min.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 左下

    p.x = max.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 左上

    p.x = min.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 右下

    p.x = max.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 上右上

    p.x = max.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 两层中间连接线

    p.x = max.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 左上

    p.x = max.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 左上

    p.x = max.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 左上

    p.x = min.x;
    p.y = max.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 两层中间连接线 底左下

    p.x = min.x;
    p.y = max.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 左上

    p.x = min.x;
    p.y = min.y;
    p.z = max.z;
    bbox_marker.points.push_back(p); // 左上

    p.x = min.x;
    p.y = min.y;
    p.z = min.z;
    bbox_marker.points.push_back(p); // 左上

    // TODO type check

    if (type)
    {
        marker_array.markers.push_back(bbox_marker);
        marker_array.markers.push_back(euc_marker);
        marker_array.markers.push_back(intensity_max_marker);
    }
    else
    {
        marker_array_all.markers.push_back(bbox_marker);
        marker_array.markers.push_back(euc_marker);
        // marker_array.markers.push_back(intensity_max_marker);
    }

    ++marker_id;
    return;
}
