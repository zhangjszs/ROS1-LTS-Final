/**
 * @file main.cpp
 * @author yzh
 * @brief Main file of High Speed Tracking, creates all modules, subcribers and publishers.
 * @version 1.0
 * @date 2022-10-31
 */

#include <autodrive_msgs/HUAT_Cone.h>
#include <autodrive_msgs/HUAT_PathLimits.h>
#include <autodrive_msgs/HUAT_TrackLimits.h>
#include <autodrive_msgs/HUAT_ConeMap.h>
#include "autodrive_msgs/HUAT_Stop.h"
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <sys/stat.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

#include "modules/DelaunayTri.hpp"
#include "modules/Visualization.hpp"
#include "modules/WayComputer.hpp"
#include "utils/Time.hpp"
#include "utils/PerfStats.hpp"
bool wasLoopClosed = false;
bool fullPathPublishedOnce = false;
int finishGraceCount = 0;
int finishGraceFrames = 300;
bool waitFullBeforeStop = true;
bool enableLoopFallbackByLapCounter = true;
int loopFallbackMinLaps = 1;
bool loopFallbackWasActive = false;

// Publishers are initialized here
ros::Publisher pubPathlimits; // 统一路径输出（partial/full 均发布到此话题）
ros::Publisher stopPub;    // 停止发布

WayComputer *wayComputer; // 使用wayComputer指针变量来访问WayComputer对象的成员函数和成员变量。
Params *params;           // 使用params指针变量来访问Params对象的成员函数和成员变量。
PerfStats perf_stats;
Visualization *visualization = nullptr;

int interTimes = 0; // 圈数
bool inter = false; // 进入判断区域

/**
 * @brief 创建一个区域，用于判断圈数，发出停止信号
 */
bool finish()
{
  // 如果没有标记为进入，并且x在范围内，那就标记为进入
  if (!inter && std::abs(wayComputer->getCarState().car_state.x) < 1 && std::abs(wayComputer->getCarState().car_state.y) < 1)
  {
    inter = true;
    std::cout << "进入判定区域" << std::endl;
  }
  // 如果标记进入，并且x>1，那么认为开始一圈
  if (std::abs(wayComputer->getCarState().car_state.y) < 1 && inter && wayComputer->getCarState().car_state.x > 1)
  {
    inter = false;
    interTimes++;
    std::cout << "开始第" << interTimes << "圈" << std::endl;
  }
  // 如果完成了xx圈（开始第xx + 1圈）就停下
  if (interTimes > params->main.number_of_stopped_turns)//
  {
    return true;
  }
  return false;
}

// 用于把路径存放到txt的函数
/*****************************test*********************************/
void doWayMsg(const autodrive_msgs::HUAT_PathLimits &msgs);
void doWayFullMsg(const autodrive_msgs::HUAT_PathLimits &msgs);
void txtClear();

void doWayMsg(const autodrive_msgs::HUAT_PathLimits &msgs)
{
  std::ofstream f;
  std::string path = ros::package::getPath("planning_ros") + "/testData/wayPartial.txt";
  f.open(path.c_str(), std::ios_base::app);
  if (f.fail())
  {
    std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl;
    return;
  }
  if (msgs.path.empty())
  {
    ROS_WARN("[path_sub] path(partial) is empty.");
    return;
  }
  std::stringstream ss;
  for (int i = 0; i < msgs.path.size(); i++)
  {
    ss << msgs.path[i].x << "\t" << msgs.path[i].y << "\t" << std::endl;
  }
  f << ss.str();
//  f << "************************************\n";
  f.close();
}

void doWayFullMsg(const autodrive_msgs::HUAT_PathLimits &msgs)
{
  std::ofstream f;
  std::string path = ros::package::getPath("planning_ros") + "/testData/wayFull.txt";
  f.open(path.c_str(), std::ios_base::app);
  if (f.fail())
  {
    std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl;
    return;
  }
  if (msgs.path.empty())
  {
    ROS_WARN("[path_sub] path(full) is empty.");
    return;
  }
  std::stringstream ss;
  for (int i = 0; i < msgs.path.size(); i++)
  {
    ss << msgs.path[i].x << "\t" << msgs.path[i].y << "\t" << std::endl;
  }
  f << ss.str();
  f << "************************************\n";
  f.close();
}

void txtClear()
{
  std::string dir = ros::package::getPath("planning_ros") + "/testData";
  mkdir(dir.c_str(), 0777);
  std::string path1 = dir + "/wayPartial.txt";
  std::string path2 = dir + "/wayFull.txt";

  std::ofstream f(path1.c_str(), std::ios::out | std::ios::trunc);
  if (f.fail())
  {
    std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl;
    return;
  }
  f.close();
  std::ofstream ff(path2.c_str(), std::ios::out | std::ios::trunc);
  if (ff.fail())
  {
    std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl;
    return;
  }
  ff.close();
}
/*****************************test*********************************/

// 这是地图回调
void callback_ccat(const autodrive_msgs::HUAT_ConeMap::ConstPtr &data)
{
  if (!params || !wayComputer || !wayComputer->isLocalTfValid())
  {
    ROS_WARN("[high_speed_tracking] CarState not being received or wayComputer is invalid.");
    return;
  }

  if (data->cone.empty())
  {
    ROS_WARN("[high_speed_tracking] Reading empty set of cones.");
    return;
  }

  ros::WallTime total_start = ros::WallTime::now();
  size_t bytes_pub = 0;




  // Time::tick("computation");  // Start measuring time

  // 把锥筒坐标转为节点，并按置信度做最小过滤
  std::vector<Node> nodes;
  nodes.reserve(data->cone.size());
  for (const autodrive_msgs::HUAT_Cone &c : data->cone)
  {
    // confidence is scaled 0–1000 in the message; decode to [0.0, 1.0] for comparison
    if (static_cast<double>(c.confidence) / 1000.0 >= params->main.min_cone_confidence)
    {
      nodes.emplace_back(c);
    }
  }

  if (nodes.empty())
  {
    ROS_WARN_THROTTLE(1.0, "[high_speed_tracking] No cones passed confidence filter (min_cone_confidence=%.3f).",
                      params->main.min_cone_confidence);
    return;
  }

  //使用节点的全局坐标和车身位姿信息计算出局部坐标,也就是说其实这个局部坐标可以不提供,反正他也是自己算出来的
  for (const Node &n : nodes)
  {
    n.updateLocal(wayComputer->getLocalTf());
  }

  //把节点传入三角剖分模块,进行三角剖分
  ros::WallTime delaunay_start = ros::WallTime::now();
  TriangleSet triangles = DelaunayTri::compute(nodes);
  ros::WallDuration delaunay_dur = ros::WallTime::now() - delaunay_start;

  // 把三角剖分结果传入路径规划模块
  ros::WallTime way_start = ros::WallTime::now();
  wayComputer->update(triangles, data->header.stamp);
  ros::WallDuration way_dur = ros::WallTime::now() - way_start;

  const bool finish_reached = finish();

  if (visualization) {
    visualization->setTimestamp(data->header.stamp);
    visualization->visualize(wayComputer->lastFilteredTriangulation());
    visualization->visualize(wayComputer->lastFilteredEdges());
    visualization->visualize(wayComputer->wayForVisualization());
  }

  // 发布循环和将轨迹限制写入文件
  const bool loop_closed_raw = wayComputer->isLoopClosed();
  const bool loop_closed_fallback =
      enableLoopFallbackByLapCounter && (interTimes >= std::max(1, loopFallbackMinLaps));
  const bool loop_fallback_active_now = (!loop_closed_raw && loop_closed_fallback);
  if (loop_fallback_active_now && !loopFallbackWasActive)
  {
    ROS_WARN("[high_speed_tracking] Loop-closure fallback activated by lap counter (interTimes=%d, threshold=%d).",
             interTimes, std::max(1, loopFallbackMinLaps));
  }
  else if (!loop_fallback_active_now && loopFallbackWasActive)
  {
    ROS_INFO("[high_speed_tracking] Loop-closure fallback cleared.");
  }
  loopFallbackWasActive = loop_fallback_active_now;
  const bool loop_closed_for_publish = loop_closed_raw || loop_closed_fallback;

  if (loop_closed_for_publish)
  {
    if (!wasLoopClosed)
    {
      ROS_INFO("[high_speed_tracking] Enter FAST_LAP: publishing full path.");
    }
    autodrive_msgs::HUAT_PathLimits full_msg = wayComputer->getPathLimitsGlobal(params->main.the_mode_of_full_path);
    if (params->main.debug_save_way_files)
    {
      doWayFullMsg(full_msg);
    }
    pubPathlimits.publish(full_msg);
    bytes_pub += ros::serialization::serializationLength(full_msg);
    fullPathPublishedOnce = true;
    finishGraceCount = 0;
    wasLoopClosed = true;
    if (params->main.debug_save_way_files)
    {
      std::string loopDir = params->main.package_path + "/loops";
      mkdir(loopDir.c_str(), 0777);
      wayComputer->writeWayToFile(loopDir + "/loop.unay");
    }
    if (params->main.shutdown_on_loop_closure)
    {
      ros::shutdown();
      return;
    }
  }
  else
  {
    if (wasLoopClosed)
    {
      ROS_WARN_THROTTLE(1.0, "[high_speed_tracking] FAST_LAP lost. Fallback to SAFE_LAP partial output.");
    }
    autodrive_msgs::HUAT_PathLimits partial_msg = wayComputer->getPathLimitsGlobal(params->main.the_mode_of_partial_path);
    if (params->main.debug_save_way_files)
    {
      doWayMsg(partial_msg);
    }
    pubPathlimits.publish(partial_msg);
    bytes_pub += ros::serialization::serializationLength(partial_msg);
    wasLoopClosed = false;
  }

  if (finish_reached)
  {
    if (waitFullBeforeStop && !fullPathPublishedOnce && finishGraceCount < std::max(0, finishGraceFrames))
    {
      ++finishGraceCount;
      ROS_WARN_THROTTLE(1.0,
                        "[high_speed_tracking] Finish reached but full path has not been published yet. "
                        "Delaying stop (%d/%d).",
                        finishGraceCount, finishGraceFrames);
    }
    else
    {
      autodrive_msgs::HUAT_Stop msg;
      msg.stop = true;
      stopPub.publish(msg);
      bytes_pub += ros::serialization::serializationLength(msg);
      std::cout << "完成" << std::endl;
      ros::shutdown();
      return;
    }
  }

  ros::WallDuration total_dur = ros::WallTime::now() - total_start;
  PerfSample sample;
  sample.t_delaunay_ms = delaunay_dur.toSec() * 1000.0;
  sample.t_way_ms = way_dur.toSec() * 1000.0;
  sample.t_total_ms = total_dur.toSec() * 1000.0;
  sample.n_points = static_cast<double>(nodes.size());
  sample.n_triangles = static_cast<double>(wayComputer->lastTriangleCount());
  sample.n_edges = static_cast<double>(wayComputer->lastEdgeCount());
  sample.bytes_pub = static_cast<double>(bytes_pub);
  perf_stats.Add(sample);
  // Time::tock("computation");  //结束测量时间
}



// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "high_speed_tracking");
  ros::NodeHandle *const nh = new ros::NodeHandle;
  ros::NodeHandle pnh("~");
  bool perf_enabled = true;
  int perf_window = 300;
  int perf_log_every = 30;
  pnh.param("wait_full_before_stop", waitFullBeforeStop, true);
  pnh.param("finish_grace_frames", finishGraceFrames, 300);
  pnh.param("enable_loop_fallback_by_lap_counter", enableLoopFallbackByLapCounter, true);
  pnh.param("loop_fallback_min_laps", loopFallbackMinLaps, 1);
  pnh.param("perf_stats_enable", perf_enabled, true);
  pnh.param("perf_stats_window", perf_window, 300);
  pnh.param("perf_stats_log_every", perf_log_every, 30);
  perf_stats.Configure("high_speed_tracking", perf_enabled, static_cast<size_t>(perf_window),
                       static_cast<size_t>(perf_log_every));
  params = new Params(nh);                                      // 加载参数
  wayComputer = new WayComputer(params->wayComputer);           // 创建一个名为wayComputer的指针，指向一个WayComputer对象，该对象的构造函数使用params->wayComputer参数来初始化。然后，使用wayComputer计算新的轨迹。
  visualization = &Visualization::getInstance();
  visualization->init(nh, params->visualization);
  if (params->main.debug_save_way_files)
  {
    txtClear();
  }
  // 初始化可视化发布器（Delaunay/中点/中心线）
  // 订阅者和发布者
  ros::Subscriber subCones = nh->subscribe(params->main.input_cones_topic, 1, callback_ccat);
  ros::Subscriber subPose = nh->subscribe(params->main.input_pose_topic, 1, &WayComputer::stateCallback, wayComputer);

  std::string pathlimits_topic;
  pnh.param<std::string>("output_pathlimits_topic", pathlimits_topic, "planning/high_speed_tracking/pathlimits");
  pubPathlimits = nh->advertise<autodrive_msgs::HUAT_PathLimits>(pathlimits_topic, 1);
  stopPub = nh->advertise<autodrive_msgs::HUAT_Stop>(params->main.stop_topic, 1);
  ros::spin();
}
