#pragma once

#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>

namespace autodrive_msgs {

/**
 * @brief Shared diagnostics publisher with rate throttling and dual local/global publish.
 *
 * Replaces the duplicated diagnostics boilerplate in control_ros, localization_ros,
 * and planning_ros (~48 lines × 3 → 3 lines × 3).
 */
class DiagnosticsHelper {
public:
  struct Config {
    std::string local_topic;
    std::string global_topic;
    bool publish_global = true;
    double rate_hz = 1.0;
    int queue_size = 1;
    bool latch_local = true;
  };

  DiagnosticsHelper() = default;

  void Init(ros::NodeHandle &nh, const Config &cfg)
  {
    cfg_ = cfg;
    if (cfg_.rate_hz <= 0.0) cfg_.rate_hz = 1.0;
    pub_local_ = nh.advertise<diagnostic_msgs::DiagnosticArray>(
        cfg_.local_topic, cfg_.queue_size, cfg_.latch_local);
    if (cfg_.publish_global)
    {
      pub_global_ = nh.advertise<diagnostic_msgs::DiagnosticArray>(
          cfg_.global_topic, cfg_.queue_size);
    }
  }

  /// Publish a pre-built DiagnosticArray (local + optional global).
  void Publish(const diagnostic_msgs::DiagnosticArray &arr)
  {
    pub_local_.publish(arr);
    if (cfg_.publish_global)
    {
      pub_global_.publish(arr);
    }
  }

  /// Rate-throttled publish. Returns false if throttled (no publish).
  bool PublishThrottled(const diagnostic_msgs::DiagnosticArray &arr, bool force = false)
  {
    const ros::Time now = ros::Time::now();
    if (!force && last_pub_.isValid())
    {
      const double min_interval = 1.0 / cfg_.rate_hz;
      if ((now - last_pub_).toSec() < min_interval)
      {
        return false;
      }
    }
    last_pub_ = now;
    Publish(arr);
    return true;
  }

  /// Convenience: build a single-status DiagnosticArray and publish with throttling.
  bool PublishStatus(const std::string &name,
                     const std::string &hardware_id,
                     uint8_t level,
                     const std::string &message,
                     const std::vector<diagnostic_msgs::KeyValue> &kvs,
                     const ros::Time &stamp = ros::Time(0),
                     bool force = false)
  {
    diagnostic_msgs::DiagnosticArray arr;
    const ros::Time now = ros::Time::now();
    arr.header.stamp = stamp.isZero() ? now : stamp;

    diagnostic_msgs::DiagnosticStatus ds;
    ds.name = name;
    ds.hardware_id = hardware_id;
    ds.level = level;
    ds.message = message;
    ds.values = kvs;

    arr.status.push_back(ds);
    return PublishThrottled(arr, force);
  }

  /// Helper to create a KeyValue pair.
  static diagnostic_msgs::KeyValue KV(const std::string &key, const std::string &value)
  {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

private:
  Config cfg_;
  ros::Publisher pub_local_;
  ros::Publisher pub_global_;
  ros::Time last_pub_;
};

}  // namespace autodrive_msgs
