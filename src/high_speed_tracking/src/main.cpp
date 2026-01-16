/**
 * @file main.cpp
 * @author yzh
 * @brief Main file of High Speed Tracking, creates all modules, subcribers and publishers.
 * @version 1.0
 * @date 2022-10-31
 */

#include <common_msgs/HUAT_cone.h>
#include <common_msgs/HUAT_PathLimits.h>
#include <common_msgs/HUAT_Tracklimits.h>
#include <common_msgs/HUAT_map.h>
#include "common_msgs/HUAT_stop.h"
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <sys/stat.h>

#include <iostream>

#include "modules/DelaunayTri.hpp"
#include "modules/Visualization.hpp"
#include "modules/WayComputer.hpp"
#include "utils/Time.hpp"
#include "utils/PerfStats.hpp"
bool wasLoopClosed = false;

// Publishers are initialized here
ros::Publisher pubPartial; // 部分路径
ros::Publisher pubFull;    // 完整路径
ros::Publisher stopPub;    // 停止发布

WayComputer *wayComputer; // 使用wayComputer指针变量来访问WayComputer对象的成员函数和成员变量。
Params *params;           // 使用params指针变量来访问Params对象的成员函数和成员变量。
PerfStats perf_stats;

int interTimes = 0; // 圈数
bool inter = false; // 进入判断区域

/**
 * @brief 创建一个区域，用于判断圈数，发出停止信号
 */
bool finish()
{
  // 如果没有标记为进入，并且x在范围内，那就标记为进入
  if (!inter && abs(wayComputer->getCarState().car_state.x) < 1 && abs(wayComputer->getCarState().car_state.y) < 1)
  {
    inter = true;
    std::cout << "进入判定区域" << std::endl;
  }
  // 如果标记进入，并且x>1，那么认为开始一圈
  if (abs(wayComputer->getCarState().car_state.y) < 1 && inter && wayComputer->getCarState().car_state.x > 1)
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
void doWayMsg(const common_msgs::HUAT_PathLimits &msgs);
void doWayFullMsg(const common_msgs::HUAT_PathLimits &msgs);
void txtClear();

void doWayMsg(const common_msgs::HUAT_PathLimits &msgs)
{
  std::ofstream f;
  std::string path = ros::package::getPath("high_speed_tracking") + "/testData/wayPartial.txt";
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

void doWayFullMsg(const common_msgs::HUAT_PathLimits &msgs)
{
  std::ofstream f;
  std::string path = ros::package::getPath("high_speed_tracking") + "/testData/wayFull.txt";
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
  std::string dir = ros::package::getPath("high_speed_tracking") + "/testData";
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
void callback_ccat(const common_msgs::HUAT_map::ConstPtr &data)
{
  if (!wayComputer || !wayComputer->isLocalTfValid())
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

  if (finish())
  {
    common_msgs::HUAT_stop msg;
    msg.stop = true;
    stopPub.publish(msg);
    bytes_pub += ros::serialization::serializationLength(msg);
    std::cout << "完成" << std::endl;
    ros::shutdown();
  }




  // Time::tick("computation");  // Start measuring time

  //把锥筒坐标转为一个一个的节点,包括锥筒的局部坐标全局坐标和唯一id,忽略置信度问题
  std::vector<Node> nodes;
  nodes.reserve(data->cone.size());
   for (const common_msgs::HUAT_cone &c : data->cone) {
    //if (c.confidence >= params->main.min_cone_confidence) 
      nodes.emplace_back(c);
  }

  //使用节点的全局坐标和车身位姿信息计算出局部坐标,也就是说其实这个局部坐标可以不提供,反正他也是自己算出来的
  for (const Node &n : nodes) {
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

  // 发布循环和将轨迹限制写入文件
    if (wayComputer->isLoopClosed()) {
  //这个用于把路径消息存到文件中，加上发布调用两次，相当于两次插值，参数5代表全部插值(局部坐标系)，反正也是环，懒得管车前车后了
    common_msgs::HUAT_PathLimits full_msg = wayComputer->getPathLimitsGlobal(3);//params->main.the_mode_of_full_path
    doWayFullMsg(full_msg);
    pubFull.publish(full_msg);//params->main.the_mode_of_full_path
    bytes_pub += ros::serialization::serializationLength(full_msg);
    // doWayMsg(wayComputer->getPathLimitsGlobal(params->main.the_mode_of_partial_path));
    // pubPartial.publish(wayComputer->getPathLimitsGlobal(params->main.the_mode_of_partial_path));
    // ROS_INFO("[high_speed_tracking] Tanco loop");
    wasLoopClosed = true;
    std::string loopDir = params->main.package_path + "/loops";
    mkdir(loopDir.c_str(), 0777);
    wayComputer->writeWayToFile(loopDir + "/loop.unay");
    //可能都没用上下面这个
    if (params->main.shutdown_on_loop_closure) {
      // Time::tock("computation");  //结束测量时间
      // ROS_INFO("[high_speed_tracking] Tingui bon dia :)");
      ros::shutdown();
    }
  } else {
    //这个用于把路径消息存到文件中，加上发布调用两次，相当于两次插值,参数4代表局部插值(局部坐标)，也就是只对车前方的路径插值
    if(!wasLoopClosed){
    common_msgs::HUAT_PathLimits partial_msg = wayComputer->getPathLimitsGlobal(2);//params->main.the_mode_of_partial_path
    doWayMsg(partial_msg);
    pubPartial.publish(partial_msg);//params->main.the_mode_of_partial_path
    bytes_pub += ros::serialization::serializationLength(partial_msg);}
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
  // 文件清空
  common_msgs::HUAT_PathLimits msgs;
  txtClear();
  ros::init(argc, argv, "high_speed_tracking");
  ros::NodeHandle *const nh = new ros::NodeHandle;
  ros::NodeHandle pnh("~");
  bool perf_enabled = true;
  int perf_window = 300;
  int perf_log_every = 30;
  pnh.param("perf_stats_enable", perf_enabled, true);
  pnh.param("perf_stats_window", perf_window, 300);
  pnh.param("perf_stats_log_every", perf_log_every, 30);
  perf_stats.Configure("high_speed_tracking", perf_enabled, static_cast<size_t>(perf_window),
                       static_cast<size_t>(perf_log_every));
  params = new Params(nh);                                      // 加载参数
  wayComputer = new WayComputer(params->wayComputer);           // 创建一个名为wayComputer的指针，指向一个WayComputer对象，该对象的构造函数使用params->wayComputer参数来初始化。然后，使用wayComputer计算新的轨迹。
  Visualization::getInstance().init(nh, params->visualization); //  //获取可视化处理  init()函数可能是用于初始化Visualization对象的状态或资源
  // 订阅者和发布者
  ros::Subscriber subCones = nh->subscribe(params->main.input_cones_topic, 1, callback_ccat);
  ros::Subscriber subPose = nh->subscribe(params->main.input_pose_topic, 1, &WayComputer::stateCallback, wayComputer);

  pubPartial = nh->advertise<common_msgs::HUAT_PathLimits>(params->main.output_partial_topic, 1);
  pubFull = nh->advertise<common_msgs::HUAT_PathLimits>(params->main.output_full_topic, 1);
  stopPub = nh->advertise<common_msgs::HUAT_stop>(params->main.stop_topic, 1);
  ros::spin();
}
