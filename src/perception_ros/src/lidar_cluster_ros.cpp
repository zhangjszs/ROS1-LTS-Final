#include <perception_ros/lidar_cluster_ros.hpp>

#include <ros/serialization.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utility>

namespace perception_ros {

LidarClusterRos::LidarClusterRos(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh))
{
  loadParams();
  core_.Configure(config_);

  sub_point_cloud_ = nh_.subscribe(input_topic_, 100, &LidarClusterRos::pointCallback, this);

  if (vis_ != 0) {
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/BoundingBox", 1);
    marker_pub_all_ = nh_.advertise<visualization_msgs::MarkerArray>("/BoundingBoxAll", 1);
  }

  pub_filtered_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/PassThrough_points", 1);
  pub_filtered_points__ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);
  pub_filtered_points___ = nh_.advertise<sensor_msgs::PointCloud2>("/SAC", 1);
  pub_filtered_points____ = nh_.advertise<sensor_msgs::PointCloud2>("/satis_filter", 1);
  lidarClusterPublisher_ = nh_.advertise<sensor_msgs::PointCloud>("/perception/lidar_cluster", 1);
  adjust_check_front_ = nh_.advertise<sensor_msgs::PointCloud2>("/adjust_check_front", 1);
  adjust_check_back_ = nh_.advertise<sensor_msgs::PointCloud2>("/adjust_check_back", 1);

  logging_pub_ = nh_.advertise<nav_msgs::Path>("/log_path", 10);
  cone_position_ = nh_.advertise<autodrive_msgs::HUAT_ConeDetections>("/cone_position", 10);
  skidpad_detection_ = nh_.advertise<sensor_msgs::PointCloud2>("/skidpad_detection", 1);

  ROS_INFO("lidar cluster finished initialization");
}

void LidarClusterRos::loadParams()
{
  if (!private_nh_.param<std::string>("input_topic", input_topic_, "/velodyne_points"))
  {
    ROS_WARN_STREAM("Did not load input_topic topic name. Standard value is: " << input_topic_);
  }
  if (!private_nh_.param<std::string>("no_ground_point_topic", no_ground_topic_, "/points_no_ground"))
  {
    ROS_WARN_STREAM("Did not load points_no_ground topic name. Standard value is: " << no_ground_topic_);
  }
  if (!private_nh_.param<std::string>("ground_point_topic", ground_topic_, "/points_ground"))
  {
    ROS_WARN_STREAM("Did not load points_ground topic name. Standard value is: " << ground_topic_);
  }
  if (!private_nh_.param<std::string>("all_points_topic", all_points_topic_, "/all_points"))
  {
    ROS_WARN_STREAM("Did not load points_ground topic name. Standard value is: " << all_points_topic_);
  }

  if (!private_nh_.param<double>("clip_height", config_.clip_height, 2.0))
  {
    ROS_WARN_STREAM("Did not load clip_height. Standard value is: " << config_.clip_height);
  }
  if (!private_nh_.param<double>("sensor_height", config_.sensor_height, 0.135))
  {
    ROS_WARN_STREAM("Did not load sensor_height. Standard value is: " << config_.sensor_height);
  }
  if (!private_nh_.param<double>("min_distance", config_.min_distance, 0.1))
  {
    ROS_WARN_STREAM("Did not load min_distance. Standard value is: " << config_.min_distance);
  }
  if (!private_nh_.param<double>("max_distance", config_.max_distance, 100.0))
  {
    ROS_WARN_STREAM("Did not load sensor_height. Standard value is: " << config_.max_distance);
  }
  if (!private_nh_.param<double>("sensor_height", config_.sensor_height, 100.0))
  {
    ROS_WARN_STREAM("Did not load sensor_height. Standard value is: " << config_.sensor_height);
  }
  if (!private_nh_.param<int>("sensor_model", config_.sensor_model, 16))
  {
    ROS_WARN_STREAM("Did not load sensor_model. Standard value is: " << config_.sensor_model);
  }
  if (!private_nh_.param<int>("num_iter", config_.num_iter, 3))
  {
    ROS_WARN_STREAM("Did not load num_iter. Standard value is: " << config_.num_iter);
  }
  if (!private_nh_.param<int>("num_lpr", config_.num_lpr, 5))
  {
    ROS_WARN_STREAM("Did not load num_lpr. Standard value is: " << config_.num_lpr);
  }
  if (!private_nh_.param<double>("th_seeds", config_.th_seeds, 0.03))
  {
    ROS_WARN_STREAM("Did not load th_seed. Standard value is: " << config_.th_seeds);
  }
  if (!private_nh_.param<double>("th_dist", config_.th_dist, 0.03))
  {
    ROS_WARN_STREAM("Did not load th_seed. Standard value is: " << config_.th_seeds);
  }
  if (!private_nh_.param<int>("road_type", config_.road_type, 2))
  {
    ROS_WARN_STREAM("Did not load road_type. Standard value is: " << config_.road_type);
  }
  if (!private_nh_.param<double>("z_up", config_.z_up, 0.7))
  {
    ROS_WARN_STREAM("Did not load z_up. Standard value is: " << config_.z_up);
  }
  if (!private_nh_.param<double>("z_down", config_.z_down, -1.0))
  {
    ROS_WARN_STREAM("Did not load road_type. Standard value is: " << config_.z_down);
  }
  if (!private_nh_.param<int>("vis", config_.vis, 0))
  {
    ROS_WARN_STREAM("Did not load vis. Standard value is: " << config_.vis);
  }
  if (!private_nh_.param<std::string>("str_range", config_.str_range, "15,30,45,60"))
  {
    ROS_WARN_STREAM("Did not load str_range. Standard value is: " << config_.str_range);
  }
  if (!private_nh_.param<std::string>("str_seg_distance", config_.str_seg_distance, "0.5,1.1,1.6,2.1,2.6"))
  {
    ROS_WARN_STREAM("Did not load str_seg_distance. Standard value is: " << config_.str_seg_distance);
  }

  if (!private_nh_.param<double>("min_height", config_.min_height, -1))
  {
    ROS_WARN("Did not load min_height.");
  }
  if (!private_nh_.param<double>("max_height", config_.max_height, -1))
  {
    ROS_WARN("Did not load max_height.");
  }
  if (!private_nh_.param<double>("min_area", config_.min_area, -1))
  {
    ROS_WARN("Did not load min_area.");
  }
  if (!private_nh_.param<double>("max_area", config_.max_area, -1))
  {
    ROS_WARN("Did not load max_area.");
  }

  if (!private_nh_.param<double>("accel_x_max", config_.accel_x_max, 0))
  {
    ROS_WARN("Did not load accel_x_max.");
  }
  if (!private_nh_.param<double>("accel_x_min", config_.accel_x_min, 0))
  {
    ROS_WARN("Did not load accel_x_min.");
  }
  if (!private_nh_.param<double>("accel_y_max", config_.accel_y_max, 0))
  {
    ROS_WARN("Did not load accel_y_max.");
  }
  if (!private_nh_.param<double>("accel_y_min", config_.accel_y_min, 0))
  {
    ROS_WARN("Did not load accel_y_min.");
  }

  if (!private_nh_.param<double>("track_x_max", config_.track_x_max, 0))
  {
    ROS_WARN("Did not load track_x_max.");
  }
  if (!private_nh_.param<double>("track_x_min", config_.track_x_min, 0))
  {
    ROS_WARN("Did not load track_x_min.");
  }
  if (!private_nh_.param<double>("track_y_max", config_.track_y_max, 0))
  {
    ROS_WARN("Did not load track_y_max.");
  }
  if (!private_nh_.param<double>("track_y_min", config_.track_y_min, 0))
  {
    ROS_WARN("Did not load track_y_min.");
  }

  if (!private_nh_.param<double>("skid_x_max", config_.skid_x_max, 0))
  {
    ROS_WARN("Did not load skid_x_max.");
  }
  if (!private_nh_.param<double>("skid_x_min", config_.skid_x_min, 0))
  {
    ROS_WARN("Did not load skid_x_min.");
  }
  if (!private_nh_.param<double>("skid_y_max", config_.skid_y_max, 0))
  {
    ROS_WARN("Did not load skid_y_max.");
  }
  if (!private_nh_.param<double>("skid_y_min", config_.skid_y_min, 0))
  {
    ROS_WARN("Did not load skid_y_min.");
  }

  if (!private_nh_.param<double>("max_box_altitude", config_.max_box_altitude, 0))
  {
    ROS_WARN("Did not load max_box_altitude.");
  }

  ROS_INFO("clip_height: %f", config_.clip_height);
  ROS_INFO("sensor_height: %f", config_.sensor_height);
  ROS_INFO("min_distance: %f", config_.min_distance);
  ROS_INFO("max_distance: %f", config_.max_distance);
  ROS_INFO("sensor_model: %d", config_.sensor_model);
  ROS_INFO("num_iter: %d", config_.num_iter);
  ROS_INFO("num_lpr: %d", config_.num_lpr);
  ROS_INFO("th_seeds: %f", config_.th_seeds);
  ROS_INFO("th_dist: %f", config_.th_dist);
  ROS_INFO("road_type: %d", config_.road_type);
  ROS_INFO("Z-axis clip limit\nUp: %f, Down: %f", config_.z_up, config_.z_down);
  ROS_INFO("vis: %d", config_.vis);

  if (!private_nh_.param<bool>("perf_stats_enable", perf_enabled_, true))
  {
    ROS_WARN_STREAM("Did not load perf_stats_enable. Standard value is: " << perf_enabled_);
  }
  int perf_window = static_cast<int>(perf_window_);
  if (!private_nh_.param<int>("perf_stats_window", perf_window, 300))
  {
    ROS_WARN_STREAM("Did not load perf_stats_window. Standard value is: " << perf_window);
  }
  perf_window_ = static_cast<size_t>(perf_window);
  int perf_log_every = static_cast<int>(perf_log_every_);
  if (!private_nh_.param<int>("perf_stats_log_every", perf_log_every, 30))
  {
    ROS_WARN_STREAM("Did not load perf_stats_log_every. Standard value is: " << perf_log_every);
  }
  perf_log_every_ = static_cast<size_t>(perf_log_every);
  perf_stats_.Configure("lidar_cluster", perf_enabled_, perf_window_, perf_log_every_);

  vis_ = config_.vis;
  sensor_model_ = config_.sensor_model;
}

void LidarClusterRos::pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*msg, *cloud);

  last_header_ = msg->header;
  got_cloud_ = true;
  core_.SetInputCloud(cloud, msg->header.seq);
}

void LidarClusterRos::RunOnce()
{
  if (!got_cloud_)
  {
    ROS_WARN_THROTTLE(5.0, "Not received Point Cloud!");
    return;
  }

  if (!core_.Process(&output_))
  {
    return;
  }

  publishOutput(output_);
}

void LidarClusterRos::publishOutput(const LidarClusterOutput &output)
{
  size_t bytes_pub = 0;

  sensor_msgs::PointCloud2 pc_msg;
  if (output.passthrough)
  {
    pcl::toROSMsg(*output.passthrough, pc_msg);
    pc_msg.header = last_header_;
    pub_filtered_points_.publish(pc_msg);
    bytes_pub += pc_msg.data.size();
  }

  if (output.not_ground)
  {
    pcl::toROSMsg(*output.not_ground, pc_msg);
    pc_msg.header = last_header_;
    pub_filtered_points___.publish(pc_msg);
    bytes_pub += pc_msg.data.size();
  }

  sensor_msgs::PointCloud2 cones_msg;
  if (output.cones_cloud)
  {
    pcl::toROSMsg(*output.cones_cloud, cones_msg);
    cones_msg.header = last_header_;
    pub_filtered_points__.publish(cones_msg);
    bytes_pub += cones_msg.data.size();
  }

  if (output.skidpad_detection)
  {
    pcl::toROSMsg(*output.skidpad_detection, pc_msg);
    pc_msg.header = last_header_;
    skidpad_detection_.publish(pc_msg);
    bytes_pub += pc_msg.data.size();
  }

  autodrive_msgs::HUAT_ConeDetections detections;
  detections.header = last_header_;
  if (output.cones_cloud)
  {
    detections.pc_whole = cones_msg;
  }

  for (const auto &det : output.cones)
  {
    geometry_msgs::Point32 center;
    center.x = det.centroid.x;
    center.y = det.centroid.y;
    center.z = det.centroid.z;
    detections.points.push_back(center);

    geometry_msgs::Point32 max;
    max.x = det.max.x;
    max.y = det.max.y;
    max.z = det.max.z;
    detections.maxPoints.push_back(max);

    geometry_msgs::Point32 min;
    min.x = det.min.x;
    min.y = det.min.y;
    min.z = det.min.z;
    detections.minPoints.push_back(min);

    if (sensor_model_ != 16)
    {
      detections.confidence.push_back(static_cast<float>(det.confidence));
    }
    detections.obj_dist.push_back(static_cast<float>(det.distance));

    if (det.cluster)
    {
      sensor_msgs::PointCloud2 cluster_msg;
      pcl::toROSMsg(*det.cluster, cluster_msg);
      cluster_msg.header = last_header_;
      detections.pc.push_back(cluster_msg);
    }
  }

  cone_position_.publish(detections);
  bytes_pub += ros::serialization::serializationLength(detections);

  if (vis_ != 0)
  {
    vis_init_status_ = visInit();
    if (vis_init_status_)
    {
      for (const auto &det : output.cones)
      {
        PointType max_pt;
        max_pt.x = det.max.x;
        max_pt.y = det.max.y;
        max_pt.z = det.max.z;
        PointType min_pt;
        min_pt.x = det.min.x;
        min_pt.y = det.min.y;
        min_pt.z = det.min.z;
        visForMarker(max_pt,
                     min_pt,
                     static_cast<float>(det.distance),
                     0,
                     0,
                     0,
                     true);
      }
      if (sensor_model_ != 16)
      {
        for (const auto &det : output.non_cones)
        {
          PointType max_pt;
          max_pt.x = det.max.x;
          max_pt.y = det.max.y;
          max_pt.z = det.max.z;
          PointType min_pt;
          min_pt.x = det.min.x;
          min_pt.y = det.min.y;
          min_pt.z = det.min.z;
          visForMarker(max_pt,
                       min_pt,
                       static_cast<float>(det.distance),
                       0,
                       0,
                       0,
                       false);
        }
        marker_pub_all_.publish(marker_array_all_);
      }
      marker_pub_.publish(marker_array_);
    }
  }

  PerfSample sample;
  sample.t_pass_ms = output.t_pass_ms;
  sample.t_ground_ms = output.t_ground_ms;
  sample.t_cluster_ms = output.t_cluster_ms;
  sample.t_total_ms = output.t_total_ms;
  sample.n_points = static_cast<double>(output.input_points);
  sample.n_clusters = static_cast<double>(output.total_clusters);
  sample.bytes_pub = static_cast<double>(bytes_pub);
  perf_stats_.Add(sample);
}

}  // namespace perception_ros
