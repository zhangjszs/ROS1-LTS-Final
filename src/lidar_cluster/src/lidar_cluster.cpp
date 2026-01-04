#include "utility.h"
#include <iomanip>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <imu_subscriber.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <distortion_adjust.hpp>
// #include <distortion_adjust.hpp>
using namespace lidar_distortion;

template <class Type>
std::string num2Str(const Type value, unsigned int precision)
{
  std::ostringstream out;
  if (precision > 0)
    out.precision(precision);
  out << value;
  return out.str();
}

// 降采样 体素滤波
void voxelFilter(const pcl::PointCloud<PointType>::Ptr input,
                 pcl::PointCloud<PointType>::Ptr &output)
{
  auto startTimePassThrough = std::chrono::steady_clock::now();
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud(input);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);                                                           // 剪枝？
  sor.filter(*output);
  auto endTimePassThrough = std::chrono::steady_clock::now();
  auto elapsedTimePassThrough = std::chrono::duration_cast<std::chrono::microseconds>(endTimePassThrough - startTimePassThrough);
  // std::cout << "VoxelGridFiltering took " << elapsedTimePassThrough.count() << " milliseconds" << std::endl;
  return;
}

lidar_cluster::lidar_cluster(ros::NodeHandle node,
                             ros::NodeHandle private_nh)
{
  // buff_mutex_ = true;
  nh_ = node;
  private_nh_ = private_nh;
  // std::cout<<"init()"<<std::endl;
  ofs.open(eval_file, std::ios::app);
  ofs.setf(std::ios::fixed, std::ios::floatfield);                                                // 存到文件里面去
  ofs.precision(5);
  
  init();
  // load parameters and init vars (def in utility.h)

  // init subscribers
  sub_point_cloud_ = nh_.subscribe(input_topic, 100, &lidar_cluster::point_callback, this);

  // init publishers
  markerPub = nh_.advertise<visualization_msgs::MarkerArray>("/BoundingBox", 1);                  // bbox of cones
  markerPubAll = nh_.advertise<visualization_msgs::MarkerArray>("/BoundingBoxAll", 1);            // bboxs that is not been identifed as cones
  // TODO rename pub_node_var
  pub_filtered_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/PassThrough_points", 1);
  pub_filtered_points__ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);
  pub_filtered_points___ = nh_.advertise<sensor_msgs::PointCloud2>("/SAC", 1);
  pub_filtered_points____ = nh_.advertise<sensor_msgs::PointCloud2>("/satis_filter", 1);
  lidarClusterPublisher_ = nh_.advertise<sensor_msgs::PointCloud>("/perception/lidar_cluster", 1);
  adjust_check_front = nh_.advertise<sensor_msgs::PointCloud2>("/adjust_check_front", 1);
  adjust_check_back = nh_.advertise<sensor_msgs::PointCloud2>("/adjust_check_back", 1);

  logging_pub = nh_.advertise<nav_msgs::Path>("/log_path", 10);
  cone_position = nh_.advertise<common_msgs::Cone>("/cone_position", 10);
  skidpad_detection = nh_.advertise<sensor_msgs::PointCloud2>("/skidpad_detection", 1);
  ROS_INFO("lidar cluster finished initialization");
}

void lidar_cluster::runAlgorithm()
{
  // string inFile = "";

  // string filename1 = "/home/adams/postsynced_msf/pcdtest/filter/pcd.pcd";
  // string filename2 = "/home/adams/postsynced_msf/pcdtest/filter/ground_out.pcd";
  // pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>());
  // pcl::PointCloud<PointType>::Ptr source2(new pcl::PointCloud<PointType>());
  // pcl::io::loadPCDFile(filename1, *source);
  // pcl::StopWatch time;

  if (!getPointClouds)
  {
    ROS_WARN("Not received Point Cloud!");
    return;
  }
  // only check once at start up

  // getPointClouds = false;
  lidar_mutex.lock();
  // *current_pc_ptr = *source;
  auto startTimePassThrough = std::chrono::steady_clock::now();
  PassThrough(current_pc_ptr, StatisticalOutlierFilter);
  cloud_filtered = current_pc_ptr;
  auto endTimePassThrough = std::chrono::steady_clock::now();
  auto elapsedTimePassThrough = std::chrono::duration_cast<std::chrono::microseconds>(endTimePassThrough - startTimePassThrough);
  // std::cout << "pass took " << elapsedTimePassThrough.count() << " milliseconds" << std::endl;

  pcl::toROSMsg(*cloud_filtered, pub_pc);
  pub_pc.header = in_pc.header;           // making headers are same

  // first: passthrough publish
  pub_filtered_points_.publish(pub_pc);

  // ground_Segmentation
  //  startTime = std::chrono::steady_clock::now();
  auto startTimeSeg = std::chrono::steady_clock::now();

  ground_segmentation(cloud_filtered, g_not_ground_pc);
  // PROSAC(cloud_filtered, g_not_ground_pc, 100, 0.05);
  // g_not_ground_pc->width = g_not_ground_pc->points.size();
  // g_not_ground_pc->height = 1;
  // pcl::io::savePCDFileASCII(filename2,*g_not_ground_pc);
  // RANSAC(cloud_filtered,g_not_ground_pc,100,0.03);
  auto endTimeSeg = std::chrono::steady_clock::now();
  auto elapsedTimeSeg = std::chrono::duration_cast<std::chrono::microseconds>(endTimeSeg - startTimeSeg);

  // endTime = std::chrono::steady_clock::now();
  // elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  // std::cout << "ground_segmentation took " << elapsedTimeSeg.count() << " milliseconds" << std::endl;
  pcl::toROSMsg(*g_not_ground_pc, pub_pc);
  pub_pc.header = in_pc.header; 
  pub_filtered_points___.publish(pub_pc); // publish not_ground_cloud

  // startTime = std::chrono::steady_clock::now();

  auto startTimeCluster = std::chrono::steady_clock::now();
  if (sensor_model_ == 16)
  {
    clusterMethod16();
  }
  else
  {
    clusterMethod32();
  }
  auto endTimeCluster = std::chrono::steady_clock::now();
  auto elapsedTimeCluster = std::chrono::duration_cast<std::chrono::microseconds>(endTimeCluster - startTimeCluster);
  // endTime = std::chrono::steady_clock::now();
  // elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  // std::cout << "cluster took " << elapsedTime.count() << " milliseconds" << std::endl;
  // lidarClusterPublisher_.publish(out_pc);
  lidar_mutex.unlock();
  auto endTime = std::chrono::steady_clock::now();
  // elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTimePassThrough);
  // std::cout << "whole took " << elapsedTime.count() << " microseconds" << std::endl;

  if (eval)
  {
    ofs << num2Str<int>(frame_count, 0) << "\t" << num2Str<float>(elapsedTimePassThrough.count(), 5)
        << "\t" << num2Str<float>(elapsedTimeSeg.count(), 5)
        << "\t" << num2Str<float>(elapsedTimeCluster.count(), 5)
        << "\t" << num2Str<float>(elapsedTime.count(), 5)
        // << "\t" << num2Str<int>(elapsedTime.count(),5)
        << std::endl;
    // ofs.close();
  }
  // frame_count+=1;
  // logging_pub.publish(ros_path_);

  // logging_pub = n.advertise<nav_msgs::Path>("log_path",10);
}
// std::ofstream out("data_test.txt",std::ios::app);

lidar_cluster::~lidar_cluster() {}
// void lidar_cluster::Spin(){}

void lidar_cluster::point_callback(const sensor_msgs::PointCloud2ConstPtr &original_cloud_ptr)
{

  // ROS_INFO_STREAM("Velodyne scan received at"<< ros::Time::now());
  // prepare for process original_cloud
  lidar_mutex.lock();
  out_pc.points.clear();
  getPointClouds = true;

  in_pc = *original_cloud_ptr; // save header
  // TODO figure out in_pc

  // std::cout<<"runAlgoIn1"<<getPointClouds<<std::endl;
  // pcl::fromROSMsg(*original_cloud_ptr, *current_pc_ptr_in);

  pcl::fromROSMsg(*original_cloud_ptr, *current_pc_ptr);    // 将原始点云传递给 current_pc_ptr

  // std::cout<<"runAlgoIn2"<<getPointClouds<<std::endl;
  out_pc.header.frame_id = "/velodyne";
  out_pc.header.stamp = original_cloud_ptr->header.stamp;
  cloud_time = original_cloud_ptr->header.stamp.toSec();
  frame_count = original_cloud_ptr->header.seq;
  lidar_mutex.unlock();
  // unsynced_imu_front=
  // disadjust->SetMotionInfo(0.1,synced_data);
  // disadjust->AdjustCloud(current_pc_ptr_in,current_pc_ptr);
  // lidar_cluster::publishToCheckAdjust(current_pc_ptr_in,current_pc_ptr);

}

// void lidar_cluster::transMatrix(const std_msgs::Float64MultiArray &msgs){
//   int element_counter = 0;
//   for (int i = 0; i < 4; i++) {
//     for (int j = 0; j < 4; j++) {
//       transMat_(i, j) = msgs.data[element_counter];
//       element_counter++;
//     }
//   }
// }

// void lidar_cluster::publishToCheckAdjust(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out)
// {
//   pcl::toROSMsg(*current_pc_ptr, pub_pc);
//   pub_pc.header = in_pc.header;
//   adjust_check_front.publish(pub_pc);

//   pcl::toROSMsg(*cloud_out, pub_pc);
//   pub_pc.header = in_pc.header;
//   adjust_check_back.publish(pub_pc);
// }
