#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <autodrive_msgs/HUAT_VisionDetections.h>
#include <fsd_common/diagnostics_helper.hpp>
#include <fsd_common/topic_contract.hpp>

#include <vision_core/types.hpp>
#include <vision_core/image_quality.hpp>
#include <vision_core/image_enhancer.hpp>
#include <vision_core/inference_backend.hpp>
#include <vision_core/fallback_detector.hpp>
#include <vision_core/temporal_tracker.hpp>
#include <vision_core/detection_postprocess.hpp>

namespace vision_ros {

class VisionNode {
public:
  enum class VisionState : uint8_t {
    NORMAL = 0,
    DEGRADED_MODE = 1,
    FALLBACK_MODE = 2,
    VISION_LOST = 3
  };

  VisionNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  void spin();

private:
  void loadParams();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void processFrame(const cv::Mat& bgr, const std_msgs::Header& header);
  void publishDetections(const std::vector<vision_core::Detection>& dets,
                         const std_msgs::Header& header,
                         const vision_core::QualityMetrics& qm,
                         uint32_t inference_us);
  void publishDebugImage(const cv::Mat& bgr,
                         const std::vector<vision_core::Detection>& dets,
                         const std_msgs::Header& header);
  void publishDiagnostics(const vision_core::QualityMetrics& qm,
                          size_t n_detections,
                          uint32_t inference_us);
  void updateState(vision_core::ImageQuality quality);
  void remapModelDetections(std::vector<vision_core::Detection>& model_dets) const;
  std::vector<vision_core::Detection> fuseDetections(
      const std::vector<vision_core::Detection>& model_dets,
      const std::vector<vision_core::Detection>& fallback_dets,
      vision_core::ImageQuality quality,
      VisionState state);

  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::Subscriber image_sub_;
  ros::Publisher detections_pub_;
  image_transport::Publisher debug_image_pub_;
  fsd_common::DiagnosticsHelper diag_helper_;

  // Core algorithm components
  std::unique_ptr<vision_core::InferenceBackend> backend_;
  std::unique_ptr<vision_core::ImageQualityAssessor> quality_assessor_;
  std::unique_ptr<vision_core::ImageEnhancer> enhancer_;
  std::unique_ptr<vision_core::FallbackDetector> fallback_;
  std::unique_ptr<vision_core::TemporalTracker> tracker_;

  // State machine
  VisionState state_ = VisionState::NORMAL;
  int consecutive_degraded_ = 0;
  int consecutive_poor_ = 0;
  int consecutive_unusable_ = 0;
  int consecutive_good_ = 0;

  // Parameters
  std::string image_topic_;
  std::string backend_type_;
  std::vector<uint8_t> class_to_color_map_;
  double loop_rate_ = 30.0;
  int max_detections_ = 20;
  float confidence_scale_ = 1000.0f;
  bool publish_debug_image_ = true;
  double debug_image_rate_ = 5.0;
  bool fallback_enabled_ = true;
  float model_confidence_floor_ = 0.3f;
  int degraded_frame_count_ = 3;
  int poor_frame_count_ = 5;
  int unusable_frame_count_ = 10;
  int recovery_frame_count_ = 5;

  // Config structs
  vision_core::InferenceConfig inference_cfg_;
  vision_core::QualityThresholds quality_thresholds_;
  vision_core::EnhancerConfig enhancer_cfg_;
  vision_core::FallbackConfig fallback_cfg_;
  vision_core::TrackerConfig tracker_cfg_;

  // Debug image throttle
  ros::Time last_debug_pub_;
  uint64_t frame_count_ = 0;
};

}  // namespace vision_ros
