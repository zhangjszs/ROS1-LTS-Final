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
#include <autodrive_msgs/topic_contract.hpp>
#include <autodrive_msgs/diagnostics_helper.hpp>
#include <fsd_common/control_mode.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
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
    pnh_.param<std::string>("cmd_topic", cmd_topic_, autodrive_msgs::topic_contract::kVehicleCmd);
    pnh_.param<std::string>("carstate_topic", carstate_topic_, autodrive_msgs::topic_contract::kCarState);
    pnh_.param<std::string>("approaching_goal_topic", approaching_goal_topic_,
                            autodrive_msgs::topic_contract::kApproachingGoal);
    pnh_.param("diagnostics_rate_hz", diagnostics_rate_hz_, 1.0);
    {
      std::string diag_topic, global_diag_topic;
      bool pub_global = true;
      pnh_.param<std::string>("diagnostics_topic", diag_topic,
                              std::string(autodrive_msgs::topic_contract::kControlDiagnostics));
      pnh_.param("publish_global_diagnostics", pub_global, true);
      pnh_.param<std::string>("global_diagnostics_topic", global_diag_topic,
                              std::string(autodrive_msgs::topic_contract::kDiagnosticsGlobal));
      autodrive_msgs::DiagnosticsHelper::Config dcfg;
      dcfg.local_topic = diag_topic;
      dcfg.global_topic = global_diag_topic;
      dcfg.publish_global = pub_global;
      dcfg.rate_hz = diagnostics_rate_hz_;
      diag_helper_.Init(nh_, dcfg);
    }
    if (diagnostics_rate_hz_ <= 0.0)
    {
      diagnostics_rate_hz_ = 1.0;
    }

    if (enable_file_mode_fallback_ || enable_external_stop_file_)
    {
      ROS_WARN("[control][deprecated][target-removal:2026-09-30] File-based compatibility path is enabled.");
    }

    external_stop_source_ = (simulation_ || !enable_external_stop_file_) ? "disabled" : "file";

    InitController();
    LoadParams();
    SetupSubscribers();

    pub_cmd_ = nh_.advertise<autodrive_msgs::HUAT_VehicleCmd>(cmd_topic_, 1000);
    ROS_INFO("[control] Command topic: %s", cmd_topic_.c_str());

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

    if (!path_ready_ && !(mode_ == fsd_common::ControlMode::kTest || mode_ == fsd_common::ControlMode::kEbs))
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
        PublishCommand(stop_cmd);
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

      PublishCommand(cmd);
      controller_->Tick();
    }
  }

private:
  void InitController()
  {
    if (mode_ == fsd_common::ControlMode::kTest)
      controller_ = std::make_unique<control_core::TestController>();
    else if (mode_ == fsd_common::ControlMode::kLine)
      controller_ = std::make_unique<control_core::LineController>();
    else if (mode_ == fsd_common::ControlMode::kSkidpad)
      controller_ = std::make_unique<control_core::SkipController>();
    else
      controller_ = std::make_unique<control_core::HighController>();
  }

  void LoadParams()
  {
    control_core::ControlParams params;
    std::string ns = (mode_ == fsd_common::ControlMode::kSkidpad) ? "st" : "pp";

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
    sub_pose_ = nh_.subscribe(carstate_topic_, 10, &ControlNode::PoseCallback, this);
    sub_last_ = nh_.subscribe(approaching_goal_topic_, 100, &ControlNode::LastCallback, this);

    ROS_INFO("[control] Subscribed car state topic: %s", carstate_topic_.c_str());
    ROS_INFO("[control] Subscribed approaching-goal topic: %s", approaching_goal_topic_.c_str());

    if (mode_ != fsd_common::ControlMode::kTest && mode_ != fsd_common::ControlMode::kEbs)
    {
      std::string pathlimits_topic;
      pnh_.param<std::string>("pathlimits_topic", pathlimits_topic,
                              autodrive_msgs::topic_contract::kPathLimits);
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

    PublishCommand(cmd);
  }

  void PublishCommand(const autodrive_msgs::HUAT_VehicleCmd &cmd)
  {
    pub_cmd_.publish(cmd);
  }

  void PublishDiagnostics(bool force)
  {
    using DH = autodrive_msgs::DiagnosticsHelper;

    uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
    std::string message = "OK";

    if (mode_source_ != "param")
    {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      message = "COMPAT_MODE_SOURCE";
    }
    if (mode_file_read_error_count_ > 0 || external_stop_file_open_error_count_ > 0)
    {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      message = "FILE_IO_WARN";
    }
    if (mode_source_ != "param" && !enable_file_mode_fallback_)
    {
      level = diagnostic_msgs::DiagnosticStatus::ERROR;
      message = "INVALID_MODE_SOURCE";
    }

    std::vector<diagnostic_msgs::KeyValue> kvs;
    kvs.push_back(DH::KV("mode_source", mode_source_));
    kvs.push_back(DH::KV("mode", std::to_string(mode_)));
    kvs.push_back(DH::KV("enable_file_mode_fallback", enable_file_mode_fallback_ ? "true" : "false"));
    kvs.push_back(DH::KV("file_mode_fallback_used", file_mode_fallback_used_ ? "true" : "false"));
    kvs.push_back(DH::KV("mode_file_path", mode_file_path_));
    kvs.push_back(DH::KV("mode_file_read_error_count", std::to_string(mode_file_read_error_count_)));
    kvs.push_back(DH::KV("external_stop_source", external_stop_source_));
    kvs.push_back(DH::KV("enable_external_stop_file", enable_external_stop_file_ ? "true" : "false"));
    kvs.push_back(DH::KV("external_stop_file_path", external_stop_file_path_));
    kvs.push_back(DH::KV("external_stop_file_open_error_count", std::to_string(external_stop_file_open_error_count_)));
    kvs.push_back(DH::KV("external_stop_trigger_count", std::to_string(external_stop_trigger_count_)));
    kvs.push_back(DH::KV("simulation", simulation_ ? "true" : "false"));
    kvs.push_back(DH::KV("cmd_topic", cmd_topic_));
    kvs.push_back(DH::KV("carstate_topic", carstate_topic_));
    kvs.push_back(DH::KV("approaching_goal_topic", approaching_goal_topic_));

    diag_helper_.PublishStatus("control_entry_health", "control_ros/control_node",
                               level, message, kvs, ros::Time(0), force);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_pathlimits_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_last_;
  ros::Publisher pub_cmd_;
  autodrive_msgs::DiagnosticsHelper diag_helper_;

  std::unique_ptr<control_core::ControllerBase> controller_;

  int mode_{0};
  int racing_num_{0};
  bool path_ready_{false};
  bool pose_ready_{false};
  bool finish_signal_{false};
  bool simulation_{false};
  std::string carstate_topic_{autodrive_msgs::topic_contract::kCarState};
  std::string approaching_goal_topic_{autodrive_msgs::topic_contract::kApproachingGoal};
  std::string cmd_topic_{autodrive_msgs::topic_contract::kVehicleCmd};

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

  double diagnostics_rate_hz_{1.0};
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
