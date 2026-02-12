#include <clocale>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_PathLimits.h>
#include <autodrive_msgs/HUAT_VehicleCmd.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/Bool.h>

#include "control_core/high_controller.hpp"
#include "control_core/line_controller.hpp"
#include "control_core/skip_controller.hpp"
#include "control_core/test_controller.hpp"

namespace control_ros
{

std::string DefaultCommandPath()
{
  const char *home = std::getenv("HOME");
  if (home == nullptr || home[0] == '\0')
  {
    return "autoStartGkj/command";
  }
  return std::string(home) + "/autoStartGkj/command";
}

int ReadModeFromFile(const std::string &path, bool *read_ok)
{
  std::ifstream ifs(path.c_str(), std::ios::in);
  if (!ifs.is_open())
  {
    if (read_ok != nullptr)
    {
      *read_ok = false;
    }
    return 0;
  }
  int value = static_cast<char>(ifs.get()) - '0';
  if (read_ok != nullptr)
  {
    *read_ok = true;
  }
  return value;
}

class ControlNode
{
public:
  ControlNode(ros::NodeHandle &nh,
              ros::NodeHandle &pnh,
              int mode,
              int racing_num,
              const std::string &mode_source,
              bool enable_file_mode_fallback,
              bool file_mode_fallback_used,
              const std::string &mode_file_path,
              int mode_file_read_error_count)
    : nh_(nh),
      pnh_(pnh),
      mode_(mode),
      racing_num_(racing_num),
      mode_source_(mode_source),
      enable_file_mode_fallback_(enable_file_mode_fallback),
      file_mode_fallback_used_(file_mode_fallback_used),
      mode_file_path_(mode_file_path),
      mode_file_read_error_count_(mode_file_read_error_count)
  {
    nh_.param("/use_sim_time", simulation_, false);
    pnh_.param("enable_external_stop_file", enable_external_stop_file_, false);
    pnh_.param<std::string>("external_stop_file_path", external_stop_file_path_, DefaultCommandPath());
    pnh_.param("diagnostics_rate_hz", diagnostics_rate_hz_, 1.0);
    pnh_.param<std::string>("diagnostics_topic", diagnostics_topic_, "control/diagnostics");
    pnh_.param("publish_global_diagnostics", publish_global_diagnostics_, true);
    pnh_.param<std::string>("global_diagnostics_topic", global_diagnostics_topic_, "/diagnostics");
    if (diagnostics_rate_hz_ <= 0.0)
    {
      diagnostics_rate_hz_ = 1.0;
    }

    external_stop_source_ = (simulation_ || !enable_external_stop_file_) ? "disabled" : "file";

    InitController();
    LoadParams();
    SetupSubscribers();

    pub_cmd_ = nh_.advertise<autodrive_msgs::HUAT_VehicleCmd>("vehcileCMDMsg", 1000);
    pub_diag_local_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 1, true);
    if (publish_global_diagnostics_)
    {
      pub_diag_global_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(global_diagnostics_topic_, 1);
    }

    PublishDiagnostics(true);
  }

  void SpinOnce()
  {
    PublishDiagnostics(false);

    if (!simulation_ && enable_external_stop_file_ && CheckExternalStop())
    {
      PublishEmergencyStop();
      ++external_stop_trigger_count_;
      PublishDiagnostics(true);
      ros::shutdown();
      return;
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

    // FSSIM风格参数
    nh_.param("car_arg/cg_to_front", params.cg_to_front, 0.77);
    nh_.param("car_arg/cg_to_rear", params.cg_to_rear, 0.78);
    nh_.param("car_arg/mass", params.mass, 190.0);
    nh_.param(ns + "/enable_slip_compensation", params.enable_slip_compensation, true);
    nh_.param(ns + "/slip_gain", params.slip_gain, 0.5);
    nh_.param(ns + "/min_lookahead", params.min_lookahead, 2.0);
    nh_.param(ns + "/max_lookahead", params.max_lookahead, 10.0);

    if (controller_)
    {
      controller_->SetParams(params);
    }
  }

  void SetupSubscribers()
  {
    sub_pose_ = nh_.subscribe("/Carstate", 10, &ControlNode::PoseCallback, this);
    sub_last_ = nh_.subscribe("/skidpad_detection_node/approaching_goal", 100, &ControlNode::LastCallback, this);

    if (mode_ != 1 && mode_ != 5)
    {
      std::string pathlimits_topic;
      pnh_.param<std::string>("pathlimits_topic", pathlimits_topic, "planning/pathlimits");
      sub_pathlimits_ = nh_.subscribe(pathlimits_topic, 100, &ControlNode::PathLimitsCallback, this);
      ROS_INFO("[control] Subscribed to pathlimits: %s", pathlimits_topic.c_str());
    }
  }

  void PoseCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &msgs)
  {
    control_core::CarState state;
    state.x = msgs->car_state.x;
    state.y = msgs->car_state.y;
    state.theta = msgs->car_state.theta;
    state.v = msgs->V;

    // FSSIM风格扩展状态 - 从IMU获取
    state.vy = msgs->Vy;           // 横向速度
    state.yaw_rate = msgs->Wz;     // 偏航角速度
    state.ax = msgs->Ax;           // 纵向加速度
    state.ay = msgs->Ay;           // 横向加速度

    if (controller_)
    {
      controller_->UpdateCarState(state);
    }
    pose_ready_ = true;
  }

  void PathLimitsCallback(const autodrive_msgs::HUAT_PathLimits::ConstPtr &msgs)
  {
    if (!pose_ready_)
    {
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
    std::ifstream ifs(external_stop_file_path_.c_str(), std::ios::in);
    if (!ifs.is_open())
    {
      ++external_stop_file_open_error_count_;
      ROS_WARN_THROTTLE(1, "文件打开失败 (External Stop Check Failed to Open File)");
      return false; // Don't stop on read error, just warn
    }
    int value = static_cast<char>(ifs.get()) - '0';
    return value == 0;
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

  void PublishDiagnostics(bool force)
  {
    const ros::Time now = ros::Time::now();
    if (!force && last_diag_pub_.isValid())
    {
      const double min_interval = 1.0 / diagnostics_rate_hz_;
      if ((now - last_diag_pub_).toSec() < min_interval)
      {
        return;
      }
    }
    last_diag_pub_ = now;

    diagnostic_msgs::DiagnosticArray diag_array;
    diag_array.header.stamp = now;

    diagnostic_msgs::DiagnosticStatus status;
    status.name = "control_entry_health";
    status.hardware_id = "control_ros/control_node";
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "OK";

    if (mode_source_ != "param")
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "COMPAT_MODE_SOURCE";
    }
    if (mode_file_read_error_count_ > 0 || external_stop_file_open_error_count_ > 0)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "FILE_IO_WARN";
    }
    if (mode_source_ != "param" && !enable_file_mode_fallback_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "INVALID_MODE_SOURCE";
    }

    auto push_kv = [&](const std::string &key, const std::string &value) {
      diagnostic_msgs::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };

    push_kv("mode_source", mode_source_);
    push_kv("mode", std::to_string(mode_));
    push_kv("enable_file_mode_fallback", enable_file_mode_fallback_ ? "true" : "false");
    push_kv("file_mode_fallback_used", file_mode_fallback_used_ ? "true" : "false");
    push_kv("mode_file_path", mode_file_path_);
    push_kv("mode_file_read_error_count", std::to_string(mode_file_read_error_count_));
    push_kv("external_stop_source", external_stop_source_);
    push_kv("enable_external_stop_file", enable_external_stop_file_ ? "true" : "false");
    push_kv("external_stop_file_path", external_stop_file_path_);
    push_kv("external_stop_file_open_error_count", std::to_string(external_stop_file_open_error_count_));
    push_kv("external_stop_trigger_count", std::to_string(external_stop_trigger_count_));
    push_kv("simulation", simulation_ ? "true" : "false");

    diag_array.status.push_back(status);
    pub_diag_local_.publish(diag_array);
    if (publish_global_diagnostics_)
    {
      pub_diag_global_.publish(diag_array);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_pathlimits_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_last_;
  ros::Publisher pub_cmd_;
  ros::Publisher pub_diag_local_;
  ros::Publisher pub_diag_global_;

  std::unique_ptr<control_core::ControllerBase> controller_;

  int mode_{0};
  int racing_num_{0};
  bool path_ready_{false};
  bool pose_ready_{false};
  bool finish_signal_{false};
  bool simulation_{false};

  std::string mode_source_{"none"};
  bool enable_file_mode_fallback_{false};
  bool file_mode_fallback_used_{false};
  std::string mode_file_path_{DefaultCommandPath()};
  int mode_file_read_error_count_{0};

  bool enable_external_stop_file_{false};
  std::string external_stop_file_path_{DefaultCommandPath()};
  std::string external_stop_source_{"disabled"};
  int external_stop_file_open_error_count_{0};
  int external_stop_trigger_count_{0};

  std::string diagnostics_topic_{"control/diagnostics"};
  bool publish_global_diagnostics_{true};
  std::string global_diagnostics_topic_{"/diagnostics"};

  double diagnostics_rate_hz_{1.0};
  ros::Time last_diag_pub_;
};

} // namespace control_ros

int main(int argc, char *argv[])
{
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "control_new");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  int mode = 0;
  std::string mode_source = "none";
  bool enable_file_mode_fallback = false;
  bool file_mode_fallback_used = false;
  std::string mode_file_path = control_ros::DefaultCommandPath();
  int mode_file_read_error_count = 0;

  private_nh.param("enable_file_mode_fallback", enable_file_mode_fallback, false);
  private_nh.param<std::string>("mode_file_path", mode_file_path, control_ros::DefaultCommandPath());

  if (private_nh.getParam("mode", mode))
  {
    ROS_INFO("Loaded mode from param: %d", mode);
    mode_source = "param";
  }
  else if (enable_file_mode_fallback)
  {
    file_mode_fallback_used = true;
    bool read_ok = false;
    mode = control_ros::ReadModeFromFile(mode_file_path, &read_ok);
    if (!read_ok)
    {
      ++mode_file_read_error_count;
      ROS_ERROR("Failed to read mode from file: %s", mode_file_path.c_str());
    }
    mode_source = read_ok ? "file" : "none";
    ROS_WARN("Loaded mode from file fallback: %d", mode);
  }
  else
  {
    ROS_ERROR("Mode is missing and file fallback is disabled. Please set private param '~mode'.");
    return 1;
  }

  if (mode == 0)
  {
    ROS_ERROR("Control mode is 0 (invalid for runtime). mode_source=%s", mode_source.c_str());
    return 1;
  }

  control_ros::ControlNode node(
      nh,
      private_nh,
      mode,
      mode,
      mode_source,
      enable_file_mode_fallback,
      file_mode_fallback_used,
      mode_file_path,
      mode_file_read_error_count);

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
