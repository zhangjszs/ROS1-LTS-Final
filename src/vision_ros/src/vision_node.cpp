#include "vision_ros/vision_node.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <diagnostic_msgs/DiagnosticStatus.h>

namespace vision_ros {

namespace tc = fsd_common::topic_contract;

static const char* stateToString(VisionNode::VisionState s) {
  switch (s) {
    case VisionNode::VisionState::NORMAL:        return "NORMAL";
    case VisionNode::VisionState::DEGRADED_MODE: return "DEGRADED";
    case VisionNode::VisionState::FALLBACK_MODE: return "FALLBACK";
    case VisionNode::VisionState::VISION_LOST:   return "VISION_LOST";
  }
  return "UNKNOWN";
}

// ── Constructor ──────────────────────────────────────────────────

VisionNode::VisionNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
  loadParams();

  // Initialize inference backend (graceful fallback if model missing)
  backend_ = vision_core::createBackend(backend_type_);
  if (backend_) {
    if (!backend_->initialize(inference_cfg_)) {
      ROS_WARN("[vision_node] Backend '%s' failed to initialize — running fallback only",
               backend_type_.c_str());
      backend_.reset();
    }
  } else {
    ROS_WARN("[vision_node] Unknown backend type '%s' — running fallback only",
             backend_type_.c_str());
  }

  // Initialize core components
  quality_assessor_ = std::make_unique<vision_core::ImageQualityAssessor>(quality_thresholds_);
  enhancer_ = std::make_unique<vision_core::ImageEnhancer>(enhancer_cfg_);
  fallback_ = std::make_unique<vision_core::FallbackDetector>(fallback_cfg_);
  tracker_ = std::make_unique<vision_core::TemporalTracker>(tracker_cfg_);

  // Publishers
  detections_pub_ = nh_.advertise<autodrive_msgs::HUAT_VisionDetections>(
      tc::kVisionDetections, 1);
  image_transport::ImageTransport it(nh_);
  debug_image_pub_ = it.advertise(tc::kVisionDebugImage, 1);

  // Diagnostics
  fsd_common::DiagnosticsHelper::Config dcfg;
  dcfg.local_topic = tc::kVisionDiagnostics;
  dcfg.global_topic = tc::kDiagnosticsGlobal;
  dcfg.rate_hz = 2.0;
  diag_helper_.Init(nh_, dcfg);

  // Subscriber
  image_transport::ImageTransport it_sub(nh_);
  image_sub_ = it_sub.subscribe(image_topic_, 1,
      &VisionNode::imageCallback, this);

  ROS_INFO("[vision_node] Initialized — backend=%s, topic=%s",
           backend_ ? backend_->backendName().c_str() : "none",
           image_topic_.c_str());
}

void VisionNode::spin() {
  ros::spin();
}

// ── Parameter Loading ────────────────────────────────────────────

void VisionNode::loadParams() {
  private_nh_.param<std::string>("image_topic", image_topic_, "camera/image_raw");
  private_nh_.param<std::string>("backend_type", backend_type_, "onnx");
  private_nh_.param<double>("loop_rate", loop_rate_, 30.0);
  private_nh_.param<int>("max_detections", max_detections_, 20);
  private_nh_.param<bool>("publish_debug_image", publish_debug_image_, true);
  private_nh_.param<double>("debug_image_rate", debug_image_rate_, 5.0);
  private_nh_.param<bool>("fallback_enabled", fallback_enabled_, true);
  private_nh_.param<float>("model_confidence_floor", model_confidence_floor_, 0.3f);
  private_nh_.param<int>("degraded_frame_count", degraded_frame_count_, 3);
  private_nh_.param<int>("poor_frame_count", poor_frame_count_, 5);
  private_nh_.param<int>("unusable_frame_count", unusable_frame_count_, 10);
  private_nh_.param<int>("recovery_frame_count", recovery_frame_count_, 5);

  // Inference config
  private_nh_.param<std::string>("model_path", inference_cfg_.model_path, "");
  private_nh_.param<int>("input_width", inference_cfg_.input_width, 640);
  private_nh_.param<int>("input_height", inference_cfg_.input_height, 640);
  float conf_thresh, nms_thresh;
  private_nh_.param<float>("conf_threshold", conf_thresh, 0.5f);
  private_nh_.param<float>("nms_threshold", nms_thresh, 0.45f);
  inference_cfg_.conf_threshold = conf_thresh;
  inference_cfg_.nms_threshold = nms_thresh;
  private_nh_.param<bool>("use_fp16", inference_cfg_.use_fp16, true);
  private_nh_.param<int>("num_threads", inference_cfg_.num_threads, 2);

  // Quality thresholds
  private_nh_.param<float>("blur_good", quality_thresholds_.blur_good, 200.0f);
  private_nh_.param<float>("blur_degraded", quality_thresholds_.blur_degraded, 100.0f);
  private_nh_.param<float>("blur_poor", quality_thresholds_.blur_poor, 50.0f);
  private_nh_.param<float>("brightness_low", quality_thresholds_.brightness_low, 40.0f);
  private_nh_.param<float>("brightness_high", quality_thresholds_.brightness_high, 220.0f);

  // Enhancer config
  private_nh_.param<bool>("auto_clahe", enhancer_cfg_.auto_clahe, true);
  float clahe_clip;
  private_nh_.param<float>("clahe_clip_limit", clahe_clip, 2.0f);
  enhancer_cfg_.clahe_clip_limit = clahe_clip;
  private_nh_.param<bool>("auto_gamma", enhancer_cfg_.auto_gamma, true);
  private_nh_.param<bool>("denoise_on_poor", enhancer_cfg_.denoise_on_poor, true);
  private_nh_.param<bool>("sharpen_on_blur", enhancer_cfg_.sharpen_on_blur, true);

  // Fallback config
  private_nh_.param<double>("fallback_min_area", fallback_cfg_.min_area, 200.0);
  private_nh_.param<double>("fallback_max_area", fallback_cfg_.max_area, 50000.0);

  // Tracker config
  float iou_thresh;
  private_nh_.param<float>("tracker_iou_threshold", iou_thresh, 0.3f);
  tracker_cfg_.iou_threshold = iou_thresh;
  private_nh_.param<int>("tracker_max_miss", tracker_cfg_.max_miss_frames, 1);
  private_nh_.param<int>("tracker_min_hits", tracker_cfg_.min_hits_to_output, 2);
}

// ── Image Callback ───────────────────────────────────────────────

void VisionNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR_THROTTLE(5.0, "[vision_node] cv_bridge exception: %s", e.what());
    return;
  }
  processFrame(cv_ptr->image, msg->header);
}

// ── Main Processing Pipeline ─────────────────────────────────────

void VisionNode::processFrame(const cv::Mat& bgr, const std_msgs::Header& header) {
  ++frame_count_;

  // 1. Assess image quality
  auto qm = quality_assessor_->assess(bgr);
  updateState(qm.overall);

  // 2. Enhance image
  cv::Mat enhanced = enhancer_->enhance(bgr, qm.overall);

  // 3. Model inference (skip if UNUSABLE)
  std::vector<vision_core::Detection> model_dets;
  uint32_t inference_us = 0;
  if (qm.overall != vision_core::ImageQuality::UNUSABLE && backend_ && backend_->isReady()) {
    auto t0 = std::chrono::steady_clock::now();
    model_dets = backend_->detect(enhanced);
    auto t1 = std::chrono::steady_clock::now();
    inference_us = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
  }

  // 4. Fallback detection
  std::vector<vision_core::Detection> fallback_dets;
  if (fallback_enabled_) {
    fallback_dets = fallback_->detect(enhanced);
  }

  // 5. Fuse detections
  auto fused = fuseDetections(model_dets, fallback_dets, qm.overall);

  // 6. Temporal tracking
  auto tracked = tracker_->update(fused);

  // 7. Top-K filter
  auto final_dets = vision_core::filterTopK(tracked, max_detections_);

  // 8. Publish
  publishDetections(final_dets, header, qm, inference_us);
  if (publish_debug_image_) {
    publishDebugImage(enhanced, final_dets, header);
  }
  publishDiagnostics(qm, final_dets.size(), inference_us);
}

// ── Detection Fusion ─────────────────────────────────────────────

std::vector<vision_core::Detection> VisionNode::fuseDetections(
    const std::vector<vision_core::Detection>& model_dets,
    const std::vector<vision_core::Detection>& fallback_dets,
    vision_core::ImageQuality quality) {

  // GOOD quality with model results: trust model only
  if (quality == vision_core::ImageQuality::GOOD && !model_dets.empty()) {
    return model_dets;
  }

  // UNUSABLE: fallback only
  if (quality == vision_core::ImageQuality::UNUSABLE) {
    return fallback_dets;
  }

  // Otherwise: merge with center-distance overlap check
  std::vector<vision_core::Detection> merged = model_dets;
  for (const auto& fb : fallback_dets) {
    bool overlaps = false;
    for (const auto& md : model_dets) {
      float dx = fb.x - md.x;
      float dy = fb.y - md.y;
      float dist = std::sqrt(dx * dx + dy * dy);
      float avg_size = (md.w + md.h + fb.w + fb.h) * 0.25f;
      if (dist < avg_size * 0.5f) {
        overlaps = true;
        break;
      }
    }
    if (!overlaps) {
      merged.push_back(fb);
    }
  }
  return merged;
}


// ── Publish Detections ───────────────────────────────────────────

void VisionNode::publishDetections(
    const std::vector<vision_core::Detection>& dets,
    const std_msgs::Header& header,
    const vision_core::QualityMetrics& qm,
    uint32_t inference_us) {

  autodrive_msgs::HUAT_VisionDetections msg;
  msg.header = header;
  msg.image_quality = static_cast<uint8_t>(qm.overall);
  msg.quality_score = std::clamp(1.0f - (qm.blur_score > 0 ? 200.0f / qm.blur_score : 0.0f)
                                  * 0.5f, 0.0f, 1.0f);

  const size_t n = dets.size();
  msg.x.resize(n);
  msg.y.resize(n);
  msg.color_types.resize(n);
  msg.confidences.resize(n);
  msg.bbox_widths.resize(n);
  msg.bbox_heights.resize(n);

  for (size_t i = 0; i < n; ++i) {
    msg.x[i] = static_cast<double>(dets[i].x);
    msg.y[i] = static_cast<double>(dets[i].y);
    msg.color_types[i] = dets[i].color_type;
    msg.confidences[i] = static_cast<int32_t>(dets[i].confidence * confidence_scale_);
    msg.bbox_widths[i] = dets[i].w;
    msg.bbox_heights[i] = dets[i].h;
  }

  bool using_fallback = (!backend_ || !backend_->isReady() ||
                         qm.overall == vision_core::ImageQuality::UNUSABLE);
  msg.backend_name = backend_ ? backend_->backendName() : "none";
  msg.fallback_active = using_fallback;
  msg.inference_time_us = inference_us;

  detections_pub_.publish(msg);
}

// ── Publish Debug Image ──────────────────────────────────────────

void VisionNode::publishDebugImage(
    const cv::Mat& bgr,
    const std::vector<vision_core::Detection>& dets,
    const std_msgs::Header& header) {

  // Throttle debug image publishing
  ros::Time now = ros::Time::now();
  if (last_debug_pub_.isValid() && debug_image_rate_ > 0.0) {
    double min_interval = 1.0 / debug_image_rate_;
    if ((now - last_debug_pub_).toSec() < min_interval) {
      return;
    }
  }
  last_debug_pub_ = now;

  if (debug_image_pub_.getNumSubscribers() == 0) {
    return;
  }

  cv::Mat canvas = bgr.clone();
  for (const auto& d : dets) {
    cv::Scalar color;
    switch (d.color_type) {
      case vision_core::BLUE:         color = cv::Scalar(255, 0, 0);     break;
      case vision_core::YELLOW:       color = cv::Scalar(0, 255, 255);   break;
      case vision_core::ORANGE_SMALL: color = cv::Scalar(0, 165, 255);   break;
      case vision_core::ORANGE_BIG:   color = cv::Scalar(0, 100, 255);   break;
      case vision_core::RED:          color = cv::Scalar(0, 0, 255);     break;
      default:                        color = cv::Scalar(200, 200, 200); break;
    }
    int x1 = static_cast<int>(d.x - d.w * 0.5f);
    int y1 = static_cast<int>(d.y - d.h * 0.5f);
    int x2 = static_cast<int>(d.x + d.w * 0.5f);
    int y2 = static_cast<int>(d.y + d.h * 0.5f);
    cv::rectangle(canvas, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
  }

  // State overlay
  cv::putText(canvas, stateToString(state_), cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

  sensor_msgs::ImagePtr out = cv_bridge::CvImage(header, "bgr8", canvas).toImageMsg();
  debug_image_pub_.publish(out);
}

// ── Publish Diagnostics ──────────────────────────────────────────

void VisionNode::publishDiagnostics(
    const vision_core::QualityMetrics& qm,
    size_t n_detections,
    uint32_t inference_us) {

  uint8_t level;
  std::string message;
  switch (state_) {
    case VisionState::NORMAL:
      level = diagnostic_msgs::DiagnosticStatus::OK;
      message = "Vision operating normally";
      break;
    case VisionState::DEGRADED_MODE:
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      message = "Vision degraded — image quality reduced";
      break;
    case VisionState::FALLBACK_MODE:
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      message = "Vision fallback — using HSV color detection";
      break;
    case VisionState::VISION_LOST:
      level = diagnostic_msgs::DiagnosticStatus::ERROR;
      message = "Vision lost — image unusable";
      break;
  }

  std::vector<diagnostic_msgs::KeyValue> kvs;
  kvs.push_back(fsd_common::DiagnosticsHelper::KV(
      "state", stateToString(state_)));
  kvs.push_back(fsd_common::DiagnosticsHelper::KV(
      "n_detections", std::to_string(n_detections)));
  kvs.push_back(fsd_common::DiagnosticsHelper::KV(
      "inference_time_us", std::to_string(inference_us)));
  kvs.push_back(fsd_common::DiagnosticsHelper::KV(
      "image_quality", std::to_string(static_cast<int>(qm.overall))));
  kvs.push_back(fsd_common::DiagnosticsHelper::KV(
      "blur_score", std::to_string(qm.blur_score)));
  kvs.push_back(fsd_common::DiagnosticsHelper::KV(
      "brightness", std::to_string(qm.brightness)));
  kvs.push_back(fsd_common::DiagnosticsHelper::KV(
      "frame_count", std::to_string(frame_count_)));
  kvs.push_back(fsd_common::DiagnosticsHelper::KV(
      "backend", backend_ ? backend_->backendName() : "none"));

  diag_helper_.PublishStatus("vision_node", "camera", level, message, kvs);
}


// ── State Machine ────────────────────────────────────────────────

void VisionNode::updateState(vision_core::ImageQuality quality) {
  // Update consecutive counters
  if (quality == vision_core::ImageQuality::GOOD) {
    consecutive_good_++;
    consecutive_degraded_ = 0;
    consecutive_poor_ = 0;
    consecutive_unusable_ = 0;
  } else if (quality == vision_core::ImageQuality::DEGRADED) {
    consecutive_degraded_++;
    consecutive_good_ = 0;
    consecutive_poor_ = 0;
    consecutive_unusable_ = 0;
  } else if (quality == vision_core::ImageQuality::POOR) {
    consecutive_poor_++;
    consecutive_good_ = 0;
    consecutive_degraded_ = 0;
    consecutive_unusable_ = 0;
  } else {
    consecutive_unusable_++;
    consecutive_good_ = 0;
    consecutive_degraded_ = 0;
    consecutive_poor_ = 0;
  }

  VisionState prev = state_;

  switch (state_) {
    case VisionState::NORMAL:
      if (consecutive_unusable_ >= unusable_frame_count_) {
        state_ = VisionState::VISION_LOST;
      } else if (consecutive_poor_ >= poor_frame_count_) {
        state_ = VisionState::FALLBACK_MODE;
      } else if (consecutive_degraded_ >= degraded_frame_count_) {
        state_ = VisionState::DEGRADED_MODE;
      }
      break;

    case VisionState::DEGRADED_MODE:
      if (consecutive_unusable_ >= unusable_frame_count_) {
        state_ = VisionState::VISION_LOST;
      } else if (consecutive_poor_ >= poor_frame_count_) {
        state_ = VisionState::FALLBACK_MODE;
      } else if (consecutive_good_ >= recovery_frame_count_) {
        state_ = VisionState::NORMAL;
      }
      break;

    case VisionState::FALLBACK_MODE:
      if (consecutive_unusable_ >= unusable_frame_count_) {
        state_ = VisionState::VISION_LOST;
      } else if (consecutive_good_ >= recovery_frame_count_) {
        state_ = VisionState::NORMAL;
      } else if (consecutive_degraded_ >= recovery_frame_count_) {
        state_ = VisionState::DEGRADED_MODE;
      }
      break;

    case VisionState::VISION_LOST:
      if (consecutive_good_ >= recovery_frame_count_) {
        state_ = VisionState::NORMAL;
      } else if (consecutive_degraded_ >= recovery_frame_count_) {
        state_ = VisionState::DEGRADED_MODE;
      } else if (consecutive_poor_ >= recovery_frame_count_) {
        state_ = VisionState::FALLBACK_MODE;
      }
      break;
  }

  if (state_ != prev) {
    ROS_INFO("[vision_node] State transition: %s -> %s",
             stateToString(prev), stateToString(state_));
  }
}

}  // namespace vision_ros
