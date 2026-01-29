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
  ofs.open(eval_file, std::ios::app);
  ofs.setf(std::ios::fixed, std::ios::floatfield);
  ofs.precision(5);
  init();
}

lidar_cluster::~lidar_cluster() = default;

void lidar_cluster::SetInputCloud(const pcl::PointCloud<PointType>::ConstPtr &cloud, uint32_t seq)
{
  if (!cloud) {
    return;
  }
  std::lock_guard<std::mutex> lock(lidar_mutex);
  *current_pc_ptr = *cloud;
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

  auto startTimePassThrough = std::chrono::steady_clock::now();
  PassThrough(current_pc_ptr);
  cloud_filtered = current_pc_ptr;
  auto endTimePassThrough = std::chrono::steady_clock::now();
  auto elapsedTimePassThrough =
      std::chrono::duration_cast<std::chrono::microseconds>(endTimePassThrough - startTimePassThrough);

  output->passthrough = cloud_filtered;

  auto startTimeSeg = std::chrono::steady_clock::now();
  ground_segmentation_dispatch_(cloud_filtered, g_not_ground_pc);
  auto endTimeSeg = std::chrono::steady_clock::now();
  auto elapsedTimeSeg =
      std::chrono::duration_cast<std::chrono::microseconds>(endTimeSeg - startTimeSeg);

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

  lock.unlock();

  auto endTimeTotal = std::chrono::steady_clock::now();
  auto elapsedTimeTotal =
      std::chrono::duration_cast<std::chrono::microseconds>(endTimeTotal - startTimePassThrough);

  output->t_pass_ms = static_cast<double>(elapsedTimePassThrough.count()) / 1000.0;
  output->t_ground_ms = static_cast<double>(elapsedTimeSeg.count()) / 1000.0;
  output->t_cluster_ms = static_cast<double>(elapsedTimeCluster.count()) / 1000.0;
  output->t_total_ms = static_cast<double>(elapsedTimeTotal.count()) / 1000.0;
  output->input_points = input_points;
  output->total_clusters = last_cluster_count_;

  if (eval) {
    ofs << num2Str<int>(frame_count, 0) << "\t" << num2Str<float>(elapsedTimePassThrough.count(), 5)
        << "\t" << num2Str<float>(elapsedTimeSeg.count(), 5)
        << "\t" << num2Str<float>(elapsedTimeCluster.count(), 5)
        << "\t" << num2Str<float>(elapsedTimeTotal.count(), 5)
        << std::endl;
  }

  return true;
}
