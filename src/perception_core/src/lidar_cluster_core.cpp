#include "perception_core/lidar_cluster_core.hpp"

#include <iomanip>
#include <sstream>

namespace {

template <class Type>
std::string num2Str(const Type value, unsigned int precision)
{
  std::ostringstream out;
  if (precision > 0) {
    out.precision(precision);
  }
  out << value;
  return out.str();
}

}  // namespace

lidar_cluster::lidar_cluster()
{
  init();
}

lidar_cluster::~lidar_cluster() = default;

void lidar_cluster::SetInputCloud(const pcl::PointCloud<PointType>::ConstPtr &cloud, uint32_t seq)
{
  if (!cloud || !current_pc_ptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(lidar_mutex);
  current_pc_ptr->clear();
  current_pc_ptr->reserve(cloud->size());
  *current_pc_ptr = *cloud;
  getPointClouds = true;
  frame_count = static_cast<int>(seq);
}

void lidar_cluster::SetInputCloud(pcl::PointCloud<PointType>::Ptr &&cloud, uint32_t seq)
{
  if (!cloud) {
    return;
  }
  std::lock_guard<std::mutex> lock(lidar_mutex);
  // 零拷贝：直接接管点云所有权，避免深拷贝
  current_pc_ptr = std::move(cloud);
  getPointClouds = true;
  frame_count = static_cast<int>(seq);
}

bool lidar_cluster::Process(LidarClusterOutput *output)
{
  if (!output) {
    return false;
  }
  if (!getPointClouds || !current_pc_ptr) {
    return false;
  }

  std::unique_lock<std::mutex> lock(lidar_mutex);
  const size_t input_points = current_pc_ptr->points.size();

  // ===== 新流水线 =====
  // ① ROI裁剪 + 强度滤波（保留原始点云密度）
  auto startTimePassThrough = std::chrono::steady_clock::now();
  PassThroughROI(current_pc_ptr);
  cloud_filtered = current_pc_ptr;
  auto endTimePassThrough = std::chrono::steady_clock::now();
  auto elapsedTimePassThrough =
      std::chrono::duration_cast<std::chrono::microseconds>(endTimePassThrough - startTimePassThrough);

  output->passthrough = cloud_filtered;

  // ② 地面分割（在原始密度点云上做，避免体素质心污染 min_z）
  auto startTimeSeg = std::chrono::steady_clock::now();
  ground_segmentation_dispatch_(cloud_filtered, g_not_ground_pc);
  auto endTimeSeg = std::chrono::steady_clock::now();
  auto elapsedTimeSeg =
      std::chrono::duration_cast<std::chrono::microseconds>(endTimeSeg - startTimeSeg);

  // ③ 降采样 + SOR（仅对非地面点，减少聚类计算量）
  PostGroundFilter(g_not_ground_pc);

  // ④ 多帧累积（远处点累积多帧提升检测率）
  AccumulateFrames(g_not_ground_pc);

  output->not_ground = g_not_ground_pc;

  auto startTimeCluster = std::chrono::steady_clock::now();
  if (sensor_model_ == 16) {
    clusterMethod16(output);
  } else {
    clusterMethod32(output);
  }
  auto endTimeCluster = std::chrono::steady_clock::now();
  auto elapsedTimeCluster =
      std::chrono::duration_cast<std::chrono::microseconds>(endTimeCluster - startTimeCluster);

  const std::size_t cluster_count = last_cluster_count_;
  lock.unlock();

  auto endTimeTotal = std::chrono::steady_clock::now();
  auto elapsedTimeTotal =
      std::chrono::duration_cast<std::chrono::microseconds>(endTimeTotal - startTimePassThrough);

  output->t_pass_ms = static_cast<double>(elapsedTimePassThrough.count()) / 1000.0;
  output->t_ground_ms = static_cast<double>(elapsedTimeSeg.count()) / 1000.0;
  output->t_cluster_ms = static_cast<double>(elapsedTimeCluster.count()) / 1000.0;
  output->t_total_ms = static_cast<double>(elapsedTimeTotal.count()) / 1000.0;
  output->input_points = input_points;
  output->total_clusters = cluster_count;

  return true;
}
