#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <utility>

#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_FusedConeDetections.h>
#include <fsd_common/topic_contract.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "perception_ros/cone_detection_adapter.hpp"

namespace perception_ros {
namespace {

class ConeDetectionAdapterNode {
 public:
  ConeDetectionAdapterNode(ros::NodeHandle nh, ros::NodeHandle pnh)
      : nh_(std::move(nh)), pnh_(std::move(pnh))
  {
    pnh_.param<std::string>("topics/raw_input", raw_input_topic_,
                            std::string(fsd_common::topic_contract::kConeDetections));
    pnh_.param<std::string>("topics/fused_input", fused_input_topic_,
                            std::string(fsd_common::topic_contract::kFusedConeDetections));
    pnh_.param<std::string>("topics/output", output_topic_,
                            std::string(fsd_common::topic_contract::kDecisionConeDetections));
    pnh_.param<std::string>("topics/trace", trace_topic_,
                            std::string(fsd_common::topic_contract::kDecisionConeTrace));
    pnh_.param<double>("flush_rate_hz", flush_rate_hz_, 100.0);
    pnh_.param<double>("summary_period_sec", summary_period_sec_, 1.0);
    pnh_.param<std::string>("contract_param_root", contract_param_root_,
                            std::string("/perception/lidar_cluster/lidar_cluster_node"));

    runtime_contract_ = LoadRuntimeContract();
    adapter_ = std::make_unique<ConeDetectionAdapter>(LoadConfig(runtime_contract_));

    raw_sub_ = nh_.subscribe(raw_input_topic_, 10, &ConeDetectionAdapterNode::rawCallback, this);
    fused_sub_ = nh_.subscribe(fused_input_topic_, 10, &ConeDetectionAdapterNode::fusedCallback, this);
    output_pub_ = nh_.advertise<autodrive_msgs::HUAT_ConeDetections>(output_topic_, 10);
    trace_pub_ = nh_.advertise<std_msgs::String>(trace_topic_, 50);
    flush_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, flush_rate_hz_)),
                                   &ConeDetectionAdapterNode::flushTimer, this);

    ROS_INFO(
        "[perception] ConeDetectionAdapter raw='%s' fused='%s' output='%s' trace='%s' mode=%s "
        "holdoff=%.3fs budget=%.3fs queue=%d finalized_ttl=%.3fs",
        raw_input_topic_.c_str(), fused_input_topic_.c_str(), output_topic_.c_str(),
        trace_topic_.c_str(),
        runtime_contract_.mode.c_str(), runtime_contract_.holdoff_sec,
        runtime_contract_.timing_budget_sec, runtime_contract_.queue_size,
        runtime_contract_.finalized_ttl_sec);
    if (runtime_contract_.contract_mismatch)
    {
      ROS_WARN(
          "[perception] ConeDetectionAdapter contract mismatch: fusion timing budget %.3fs "
          "exceeds adapter holdoff %.3fs",
          runtime_contract_.timing_budget_sec, runtime_contract_.holdoff_sec);
    }
  }

 private:
  struct RuntimeContract {
    std::string mode = "raw-only";
    double timing_budget_sec = 0.0;
    double holdoff_sec = 0.0;
    double finalized_ttl_sec = 2.0;
    int queue_size = 0;
    bool contract_mismatch = false;
  };

  RuntimeContract LoadRuntimeContract() const
  {
    RuntimeContract contract;

    double raw_holdoff_override_sec = -1.0;
    pnh_.param<double>("raw_holdoff_sec", raw_holdoff_override_sec, -1.0);
    pnh_.param<double>("finalized_ttl_sec", contract.finalized_ttl_sec, 2.0);

    const std::string fusion_enabled_param = contract_param_root_ + "/fusion/enabled";
    const std::string fusion_slop_param = contract_param_root_ + "/fusion/sync/slop_sec";
    const std::string fusion_queue_param = contract_param_root_ + "/fusion/sync/queue_size";
    const std::string legacy_enabled_param = contract_param_root_ + "/vision_inject/enabled";
    const std::string legacy_budget_param = contract_param_root_ + "/vision_inject/max_age_sec";

    bool fusion_enabled = false;
    bool legacy_enabled = false;
    if (ros::param::has(fusion_enabled_param))
    {
      ros::param::param(fusion_enabled_param, fusion_enabled, false);
      ros::param::param(fusion_slop_param, contract.timing_budget_sec, 0.08);
      ros::param::param(fusion_queue_param, contract.queue_size, 20);
      contract.mode = fusion_enabled ? "calibrated" : "raw-only";
    }
    else
    {
      ros::param::param(legacy_enabled_param, legacy_enabled, false);
      ros::param::param(legacy_budget_param, contract.timing_budget_sec, 0.15);
      contract.queue_size = legacy_enabled ? 20 : 0;
      contract.mode = legacy_enabled ? "legacy" : "raw-only";
    }

    if (contract.mode == "raw-only")
    {
      contract.timing_budget_sec = 0.0;
    }

    contract.holdoff_sec =
        raw_holdoff_override_sec >= 0.0 ? raw_holdoff_override_sec : contract.timing_budget_sec;
    contract.contract_mismatch = contract.timing_budget_sec > contract.holdoff_sec + 1e-6;
    return contract;
  }

  ConeDetectionAdapter::Config LoadConfig(const RuntimeContract &contract) const
  {
    ConeDetectionAdapter::Config cfg;
    int max_pending = 100;
    pnh_.param<int>("max_pending", max_pending, 100);
    cfg.raw_holdoff = ros::Duration(std::max(0.0, contract.holdoff_sec));
    cfg.finalized_ttl = ros::Duration(std::max(0.0, contract.finalized_ttl_sec));
    cfg.max_pending = static_cast<std::size_t>(std::max(1, max_pending));
    return cfg;
  }

  void publishOutputs(const std::vector<ConeDetectionAdapter::PublishOutput> &outputs)
  {
    for (const auto &output : outputs)
    {
      output_pub_.publish(output.msg);
      publishTrace(output);
    }
  }

  void publishTrace(const ConeDetectionAdapter::PublishOutput &output)
  {
    std_msgs::String trace_msg;
    std::ostringstream trace;
    trace << "stamp_ns=" << output.msg.header.stamp.toNSec()
          << " publish_mode=" << (output.used_fused ? "fused" : "raw_fallback")
          << " reason=" << output.reason
          << " wait_ms=" << output.wait_ms;
    trace_msg.data = trace.str();
    trace_pub_.publish(trace_msg);
  }

  bool HasActivity(const ConeDetectionAdapter::Stats &stats) const
  {
    return stats.published_fused > 0 || stats.published_raw_fallback > 0 || stats.no_fused > 0 ||
        stats.late_fused > 0 || stats.stale_fused > 0 || stats.count_mismatch > 0 ||
        stats.cache_evicted > 0 || stats.duplicate_after_finalize > 0;
  }

  void maybeLogSummary() const
  {
    const auto &stats = adapter_->stats();
    if (!HasActivity(stats))
    {
      return;
    }

    ROS_INFO_THROTTLE(
        std::max(0.1, summary_period_sec_),
        "[perception] ConeDetectionAdapter stats fused=%zu raw_fallback=%zu no_fused=%zu "
        "late_fused=%zu stale_fused=%zu count_mismatch=%zu cache_evicted=%zu "
        "duplicate_after_finalize=%zu pending_cache_size_max=%zu",
        stats.published_fused, stats.published_raw_fallback, stats.no_fused, stats.late_fused,
        stats.stale_fused, stats.count_mismatch, stats.cache_evicted,
        stats.duplicate_after_finalize, stats.pending_cache_size_max);
  }

  void rawCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &msg)
  {
    publishOutputs(adapter_->HandleRaw(*msg, ConeDetectionAdapter::SteadyClock::now()));
    maybeLogSummary();
  }

  void fusedCallback(const autodrive_msgs::HUAT_FusedConeDetections::ConstPtr &msg)
  {
    publishOutputs(adapter_->HandleFused(*msg, ConeDetectionAdapter::SteadyClock::now()));
    maybeLogSummary();
  }

  void flushTimer(const ros::TimerEvent &event)
  {
    (void)event;
    publishOutputs(adapter_->Flush(ConeDetectionAdapter::SteadyClock::now()));
    maybeLogSummary();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::unique_ptr<ConeDetectionAdapter> adapter_;
  RuntimeContract runtime_contract_;
  std::string raw_input_topic_;
  std::string fused_input_topic_;
  std::string output_topic_;
  std::string trace_topic_;
  std::string contract_param_root_;
  double flush_rate_hz_ = 100.0;
  double summary_period_sec_ = 1.0;
  ros::Subscriber raw_sub_;
  ros::Subscriber fused_sub_;
  ros::Publisher output_pub_;
  ros::Publisher trace_pub_;
  ros::Timer flush_timer_;
};

}  // namespace
}  // namespace perception_ros

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cone_detection_adapter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  perception_ros::ConeDetectionAdapterNode node(nh, pnh);
  ros::spin();
  return 0;
}
