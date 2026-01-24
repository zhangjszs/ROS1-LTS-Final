#include <clocale>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_PathLimits.h>
#include <autodrive_msgs/HUAT_VehicleCmd.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include "control_core/high_controller.hpp"
#include "control_core/line_controller.hpp"
#include "control_core/skip_controller.hpp"
#include "control_core/test_controller.hpp"

namespace control_ros
{

class ControlNode
{
public:
  ControlNode(ros::NodeHandle &nh, ros::NodeHandle &pnh, int mode, int racing_num)
    : nh_(nh), pnh_(pnh), mode_(mode), racing_num_(racing_num)
  {
    InitController();
    LoadParams();
    SetupSubscribers();

    pub_cmd_ = nh_.advertise<autodrive_msgs::HUAT_VehicleCmd>("vehcileCMDMsg", 1000);
  }

  void SpinOnce()
  {
    if (CheckExternalStop())
    {
      PublishEmergencyStop();
      std::exit(0);
    }

    if (!path_ready_ && !(mode_ == 1 || mode_ == 5))
    {
      ROS_WARN("得不到有效的惯导路径信息");
      return;
    }

    if (controller_)
    {
      auto output = controller_->ComputeOutput();
      if (output.stop_requested)
      {
        autodrive_msgs::HUAT_VehicleCmd stop_cmd;
        pub_cmd_.publish(stop_cmd);
        ros::shutdown();
        return;
      }

      autodrive_msgs::HUAT_VehicleCmd cmd;
      cmd.head1 = 0XAA;
      cmd.head2 = 0X55;
      cmd.length = 10;

      cmd.steering = output.steering;
      cmd.pedal_ratio = output.pedal_ratio;
      cmd.brake_force = output.brake_force;
      cmd.gear_position = 1;

      cmd.working_mode = 1;
      cmd.racing_num = racing_num_;
      cmd.racing_status = output.racing_status;
      cmd.checksum = cmd.head1 + cmd.head2 + cmd.length +
                     cmd.steering + cmd.pedal_ratio + cmd.brake_force +
                     cmd.gear_position + cmd.working_mode + cmd.racing_num + cmd.racing_status;

      pub_cmd_.publish(cmd);
      controller_->Tick();
    }
  }

private:
  void InitController()
  {
    if (mode_ == 1)
      controller_ = std::make_unique<control_core::TestController>();
    else if (mode_ == 2)
      controller_ = std::make_unique<control_core::LineController>();
    else if (mode_ == 3)
      controller_ = std::make_unique<control_core::SkipController>();
    else
      controller_ = std::make_unique<control_core::HighController>();
  }

  void LoadParams()
  {
    control_core::ControlParams params;
    std::string ns = (mode_ == 3) ? "st" : "pp";

    nh_.param(ns + "/angle_kp", params.angle_kp, 1.0);
    nh_.param(ns + "/angle_ki", params.angle_ki, 0.0);
    nh_.param(ns + "/angle_kd", params.angle_kd, 0.0);
    nh_.param(ns + "/steering_delta_max", params.steering_delta_max, 0.5);
    nh_.param(ns + "/angle_kv", params.angle_kv, 0.0);
    nh_.param(ns + "/angle_kl", params.angle_kl, 2.0);
    nh_.param("car_arg/length", params.car_length, 1.55);

    if (controller_)
    {
      controller_->SetParams(params);
    }
  }

  void SetupSubscribers()
  {
    sub_pose_ = nh_.subscribe("/Carstate", 10, &ControlNode::PoseCallback, this);
    sub_last_ = nh_.subscribe("/skidpad_detection_node/approaching_goal", 100, &ControlNode::LastCallback, this);

    if (mode_ == 2)
    {
      sub_path_ = nh_.subscribe("/line_creat/line_global_path", 100, &ControlNode::PathCallback, this);
    }
    else if (mode_ == 3)
    {
      sub_path_ = nh_.subscribe("/skidpad_detection_node/log_path", 100, &ControlNode::PathCallback, this);
    }
    else if (mode_ != 1)
    {
      sub_high_path_ = nh_.subscribe("/AS/P/pathlimits/partial", 100, &ControlNode::HighPathCallback, this);
    }
  }

  void PoseCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &msgs)
  {
    control_core::CarState state;
    state.x = msgs->car_state.x;
    state.y = msgs->car_state.y;
    state.theta = msgs->car_state.theta;
    state.v = msgs->V;

    if (controller_)
    {
      controller_->UpdateCarState(state);
    }
    pose_ready_ = true;
  }

  void PathCallback(const nav_msgs::Path::ConstPtr &msgs)
  {
    if (!pose_ready_)
    {
      ROS_WARN("CarState not being received.");
      return;
    }
    pose_ready_ = false;

    std::vector<control_core::Position> path;
    path.reserve(msgs->poses.size());
    for (const auto &pose : msgs->poses)
    {
      control_core::Position pt;
      pt.x = pose.pose.position.x;
      pt.y = pose.pose.position.y;
      path.push_back(pt);
    }

    if (controller_)
    {
      controller_->UpdatePath(path);
    }
    path_ready_ = true;
  }

  void HighPathCallback(const autodrive_msgs::HUAT_PathLimits::ConstPtr &msgs)
  {
    if (!pose_ready_)
    {
      ROS_WARN("CarState not being received.");
      return;
    }
    pose_ready_ = false;

    std::vector<control_core::Position> path;
    path.reserve(msgs->path.size());
    for (const auto &path_tem : msgs->path)
    {
      control_core::Position pt;
      pt.x = path_tem.x;
      pt.y = path_tem.y;
      path.push_back(pt);
    }

    if (controller_)
    {
      controller_->UpdatePath(path);
    }
    path_ready_ = true;
  }

  void LastCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    finish_signal_ = msg->data;
    if (controller_)
    {
      controller_->SetFinishSignal(finish_signal_);
    }
  }

  bool CheckExternalStop()
  {
    std::ifstream ifs;
    std::string home_path = std::getenv("HOME") ? std::getenv("HOME") : std::string();
    std::string cmd_path = home_path + "/autoStartGkj/command";
    ifs.open(cmd_path.c_str(), std::ios::in);
    if (!ifs.is_open())
    {
      std::cout << "文件打开失败" << std::endl;
      return true;
    }
    int value = static_cast<char>(ifs.get()) - '0';
    if (value == 0)
    {
      return true;
    }
    return false;
  }

  void PublishEmergencyStop()
  {
    autodrive_msgs::HUAT_VehicleCmd cmd;
    cmd.head1 = 0XAA;
    cmd.head2 = 0X55;
    cmd.length = 10;
    cmd.steering = 110;
    cmd.pedal_ratio = 0;
    cmd.brake_force = 80;
    cmd.gear_position = 0;
    cmd.working_mode = 1;
    cmd.racing_num = racing_num_;
    cmd.racing_status = 5;
    cmd.checksum = cmd.head1 + cmd.head2 + cmd.length +
                   cmd.steering + cmd.pedal_ratio + cmd.brake_force +
                   cmd.gear_position + cmd.working_mode + cmd.racing_num + cmd.racing_status;

    pub_cmd_.publish(cmd);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_path_;
  ros::Subscriber sub_high_path_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_last_;
  ros::Publisher pub_cmd_;

  std::unique_ptr<control_core::ControllerBase> controller_;

  int mode_{0};
  int racing_num_{0};
  bool path_ready_{false};
  bool pose_ready_{false};
  bool finish_signal_{false};
};

int GetModeFromFile()
{
  std::ifstream ifs;
  int num = 0;
  while (!num)
  {
    std::string home_path = std::getenv("HOME") ? std::getenv("HOME") : std::string();
    std::string cmd_path = home_path + "/autoStartGkj/command";
    ifs.open(cmd_path.c_str(), std::ios::in);
    if (!ifs.is_open())
    {
      std::cout << "文件打开失败" << std::endl;
      return 0;
    }
    num = static_cast<char>(ifs.get()) - '0';
    ifs.close();
  }
  return num;
}

} // namespace control_ros

int main(int argc, char *argv[])
{
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "control_new");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  int mode = 0;
  if (private_nh.getParam("mode", mode))
  {
    ROS_INFO("Loaded mode from param: %d", mode);
  }
  else
  {
    mode = control_ros::GetModeFromFile();
  }

  if (mode == 0)
  {
    return 0;
  }

  control_ros::ControlNode node(nh, private_nh, mode, mode);

  ros::Rate rate(10);
  ros::Duration(1).sleep();
  while (ros::ok())
  {
    ros::spinOnce();
    node.SpinOnce();
    rate.sleep();
  }

  return 0;
}
