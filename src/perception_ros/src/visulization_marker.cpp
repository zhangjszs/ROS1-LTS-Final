#include <perception_ros/lidar_cluster_ros.hpp>

namespace perception_ros {

bool LidarClusterRos::visInit()
{
    marker_array_.markers.clear();
    marker_array_all_.markers.clear();
    marker_id_ = 0;
    euc_marker_.header.frame_id = "velodyne";
    euc_marker_.header.stamp = ros::Time::now();
    euc_marker_.ns = "euc";
    euc_marker_.color.r = 1.0f;
    euc_marker_.color.g = 0.0f;
    euc_marker_.color.b = 0.0f;
    euc_marker_.color.a = 1.0f;
    euc_marker_.lifetime = ros::Duration(0.1);
    euc_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    euc_marker_.action = visualization_msgs::Marker::ADD;

    bbox_marker_.header.frame_id = "velodyne";
    bbox_marker_.header.stamp = ros::Time::now();
    bbox_marker_.ns = "lines";
    bbox_marker_.color.r = 1.0f;
    bbox_marker_.color.g = 1.0f;
    bbox_marker_.color.b = 1.0f;
    bbox_marker_.color.a = 1.0f;
    bbox_marker_.lifetime = ros::Duration(0.1);
    bbox_marker_.scale.x = 0.01;
    bbox_marker_.type = visualization_msgs::Marker::LINE_LIST;
    bbox_marker_.action = visualization_msgs::Marker::ADD;
    return true;
}

void LidarClusterRos::visForMarker(const PointType max,
                                  const PointType min,
                                  float euc,
                                  float intensity_max,
                                  float intensity_min,
                                  float intensity_mean,
                                  bool type)
{
    (void)intensity_max;
    (void)intensity_min;
    (void)intensity_mean;

    bbox_marker_.points.clear();
    bbox_marker_.id = marker_id_;
    euc_marker_.id = marker_id_;

    euc_marker_.pose.position.x = (max.x + min.x) / 2;
    euc_marker_.pose.position.y = (max.y + min.y) / 2;
    euc_marker_.pose.position.z = (max.z + min.z) / 2 + 0.5;

    std::string text = std::to_string(euc);
    euc_marker_.text = text;
    euc_marker_.scale.z = 1;

    p_.x = min.x;
    p_.y = max.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 左下

    p_.x = min.x;
    p_.y = min.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 右下

    p_.x = max.x;
    p_.y = max.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 左上

    p_.x = max.x;
    p_.y = min.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 右上

    p_.x = min.x;
    p_.y = max.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 左下

    p_.x = max.x;
    p_.y = max.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 左上

    p_.x = min.x;
    p_.y = min.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 右下

    p_.x = max.x;
    p_.y = min.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 底右上

    p_.x = min.x;
    p_.y = max.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 左下

    p_.x = min.x;
    p_.y = min.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 右下

    p_.x = max.x;
    p_.y = max.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 左上

    p_.x = max.x;
    p_.y = min.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 右上

    p_.x = min.x;
    p_.y = max.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 左下

    p_.x = max.x;
    p_.y = max.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 左上

    p_.x = min.x;
    p_.y = min.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 右下

    p_.x = max.x;
    p_.y = min.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 上右上

    p_.x = max.x;
    p_.y = max.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 两层中间连接线

    p_.x = max.x;
    p_.y = max.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 左上

    p_.x = max.x;
    p_.y = min.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 左上

    p_.x = max.x;
    p_.y = min.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 左上

    p_.x = min.x;
    p_.y = max.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 两层中间连接线 底左下

    p_.x = min.x;
    p_.y = max.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 左上

    p_.x = min.x;
    p_.y = min.y;
    p_.z = max.z;
    bbox_marker_.points.push_back(p_); // 左上

    p_.x = min.x;
    p_.y = min.y;
    p_.z = min.z;
    bbox_marker_.points.push_back(p_); // 左上

    if (type)
    {
        marker_array_.markers.push_back(bbox_marker_);
        marker_array_.markers.push_back(euc_marker_);
        marker_array_.markers.push_back(intensity_max_marker_);
    }
    else
    {
        marker_array_all_.markers.push_back(bbox_marker_);
        marker_array_.markers.push_back(euc_marker_);
    }

    ++marker_id_;
    return;
}

}  // namespace perception_ros
