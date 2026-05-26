#include "fsd_visualization/cone_visualizer.hpp"

#include <algorithm>
#include <cstdint>

#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace fsd_viz {

namespace {
// 新锥桶类型定义（已移除橙色，改为大小黄色）
// 0=BLUE, 1=YELLOW_SMALL, 2=YELLOW_BIG, 3=RED, 4=NONE
constexpr std::uint8_t kConeBlue = 0;
constexpr std::uint8_t kConeYellowSmall = 1;
constexpr std::uint8_t kConeYellowBig = 2;
constexpr std::uint8_t kConeRed = 3;
constexpr std::uint8_t kConeNone = 4;
constexpr std::uint8_t kFusionMatched = 0;
constexpr std::uint8_t kFusionLegacyHfovMatched = 8;

// 旧类型常量（用于识别遗留数据）
// 旧定义: 0=BLUE, 1=YELLOW, 2=ORANGE_SMALL, 3=ORANGE_BIG, 4=NONE, 5=RED
constexpr std::uint8_t kLegacyYellow = 1;       // 旧黄色 -> 新小黄色
constexpr std::uint8_t kLegacyOrangeSmall = 2;  // 旧小橙色 -> 新大黄色
constexpr std::uint8_t kLegacyOrangeBig = 3;    // 旧大橙色 -> 新大黄色
constexpr std::uint8_t kLegacyNone = 4;         // 保持不变
constexpr std::uint8_t kLegacyRed = 5;          // 旧红色 -> 新红色

int normalizeConeType(int raw_type) {
  // 首先检查是否是新类型范围内的值
  if (raw_type == kConeBlue || raw_type == kConeYellowSmall || raw_type == kConeYellowBig ||
      raw_type == kConeRed || raw_type == kConeNone) {
    return raw_type;
  }

  // 处理遗留类型值（旧消息可能包含的值）
  switch (raw_type) {
    case kLegacyYellow:
      return kConeYellowSmall;  // 旧黄色 -> 新小黄色
    case kLegacyOrangeSmall:
    case kLegacyOrangeBig:
      return kConeYellowBig;  // 旧橙色 -> 新大黄色
    case kLegacyRed:
      return kConeRed;  // 旧红色 -> 新红色
    case kLegacyNone:
      return kConeNone;  // 保持不变
    default:
      return kConeNone;  // 未知类型 -> NONE
  }
}

int parseLegacyColorChar(char c) {
  if (c == 'b' || c == 'B')
    return kConeBlue;
  if (c == 'y' || c == 'Y')
    return kConeYellowSmall;  // 黄色统一映射到小黄色
  if (c == 'r' || c == 'R')
    return kConeRed;
  // 旧橙色字符映射到黄色（向后兼容）
  if (c == 'o' || c == 'O')
    return kConeYellowSmall;
  return kConeNone;
}
}  // namespace

ConeVisualizer::ConeVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : tf_listener_(tf_buffer_), tf_lookup_failure_count_(0), tf_extrapolation_failure_count_(0) {
  // 参数
  pnh.param<std::string>("frame_id", frame_id_, FRAME_GLOBAL);
  pnh.param<double>("cone_radius", cone_radius_, CONE_RADIUS);
  pnh.param<double>("cone_height", cone_height_, CONE_HEIGHT);
  pnh.param<bool>("show_bounding_box", show_bounding_box_, true);
  pnh.param<bool>("show_distance_label", show_distance_label_, true);
  pnh.param<bool>("use_mesh", use_mesh_, true);
  pnh.param<bool>("use_confidence_alpha", use_confidence_alpha_, true);
  pnh.param<bool>("publish_detection_markers", publish_detection_markers_, true);
  pnh.param<bool>("cone/prefer_fused_detections", prefer_fused_detections_, true);
  pnh.param<bool>("cone/unmatched_as_white", unmatched_as_white_, true);
  pnh.param<double>("cone/fused_stale_timeout_sec", fused_stale_timeout_sec_, 0.35);
  pnh.param<float>("min_alpha", min_alpha_, 0.3f);
  pnh.param<std::string>("cone_mesh_type", cone_mesh_type_, "gazebo");
  pnh.param<std::string>("topics/cone_detections", cone_detections_topic_,
                         autodrive_msgs::topic_contract::kConeDetections);
  pnh.param<std::string>("topics/fused_cone_detections", fused_cone_detections_topic_,
                         autodrive_msgs::topic_contract::kFusedConeDetections);
  pnh.param<std::string>("topics/cone_map", cone_map_topic_,
                         autodrive_msgs::topic_contract::kConeMap);
  pnh.param<std::string>("topics/markers", markers_topic_, "fsd/viz/cones");
  last_detection_source_ = "none";
  last_fused_matched_count_ = 0;
  last_fused_unmatched_count_ = 0;

  // B8: TF timeout monitoring parameters
  pnh.param<double>("tf_timeout_sec", tf_timeout_sec_, 0.1);

  // B8: Initialize diagnostics helper
  fsd_common::DiagnosticsHelper::Config diag_cfg;
  diag_cfg.local_topic = "fsd_viz/diagnostics";
  diag_cfg.global_topic = "/diagnostics";
  diag_cfg.publish_global = true;
  diag_cfg.rate_hz = 1.0;
  diag_helper_.Init(nh, diag_cfg);

  // 订阅
  sub_cone_detections_ =
      nh.subscribe(cone_detections_topic_, 1, &ConeVisualizer::coneDetectionsCallback, this);
  sub_fused_cone_detections_ = nh.subscribe(fused_cone_detections_topic_, 1,
                                            &ConeVisualizer::fusedConeDetectionsCallback, this);
  sub_cone_map_ = nh.subscribe(cone_map_topic_, 1, &ConeVisualizer::coneMapCallback, this);

  // 发布
  // Latch latest markers so RViz opened later can still render world objects.
  pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>(markers_topic_, 1, true);

  // Runtime color overrides
  cone_bbox_color_ = loadColorParam(pnh, "colors/cone_bbox", {1.0f, 1.0f, 1.0f, 0.8f});
  cone_label_color_ = loadColorParam(pnh, "colors/cone_label", {1.0f, 0.2f, 0.2f, 1.0f});

  // B8: Diagnostics timer (1Hz)
  diag_timer_ =
      nh.createTimer(ros::Duration(1.0), [this](const ros::TimerEvent&) { publishDiagnostics(); });

  ROS_INFO(
      "[ConeVisualizer] Initialized (tf_timeout=%.3f sec, prefer_fused=%s, "
      "unmatched_as_white=%s)",
      tf_timeout_sec_, prefer_fused_detections_ ? "true" : "false",
      unmatched_as_white_ ? "true" : "false");
  ROS_INFO("[ConeVisualizer] topics: raw='%s' fused='%s' map='%s'", cone_detections_topic_.c_str(),
           fused_cone_detections_topic_.c_str(), cone_map_topic_.c_str());
}

void ConeVisualizer::coneDetectionsCallback(
    const autodrive_msgs::HUAT_ConeDetections::ConstPtr& msg) {
  if (!publish_detection_markers_) {
    return;
  }
  if (prefer_fused_detections_ && last_fused_msg_time_.isValid()) {
    const double fused_age = (ros::Time::now() - last_fused_msg_time_).toSec();
    if (fused_age <= fused_stale_timeout_sec_) {
      return;
    }
  }
  last_detection_source_ = "raw";

  visualization_msgs::MarkerArray markers;
  const std::string source_frame = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;
  const bool need_transform = source_frame != frame_id_;
  std::string marker_frame = source_frame;
  geometry_msgs::TransformStamped tf_msg;

  if (need_transform) {
    try {
      // B8: Use configurable timeout instead of hardcoded 0.05
      tf_msg = tf_buffer_.lookupTransform(frame_id_, source_frame, msg->header.stamp,
                                          ros::Duration(tf_timeout_sec_));
    } catch (const tf2::TransformException& ex) {
      // B8: Track extrapolation failures
      if (std::string(ex.what()).find("extrapolation") != std::string::npos) {
        ++tf_extrapolation_failure_count_;
      }
      try {
        tf_msg = tf_buffer_.lookupTransform(frame_id_, source_frame, ros::Time(0),
                                            ros::Duration(tf_timeout_sec_));
      } catch (const tf2::TransformException& ex2) {
        ++tf_lookup_failure_count_;
        last_tf_failure_time_ = ros::Time::now();
        ROS_WARN_THROTTLE(1.0, "[ConeVisualizer] TF %s->%s unavailable for detections: %s",
                          source_frame.c_str(), frame_id_.c_str(), ex2.what());
        return;
      }
    }
    marker_frame = frame_id_;
  }

  auto transform_point = [&](const geometry_msgs::Point32& in,
                             geometry_msgs::Point32& out) -> bool {
    if (!need_transform) {
      out = in;
      return true;
    }
    geometry_msgs::PointStamped in_stamped;
    geometry_msgs::PointStamped out_stamped;
    in_stamped.header.frame_id = source_frame;
    in_stamped.header.stamp = msg->header.stamp;
    in_stamped.point.x = in.x;
    in_stamped.point.y = in.y;
    in_stamped.point.z = in.z;
    try {
      tf2::doTransform(in_stamped, out_stamped, tf_msg);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "[ConeVisualizer] Point transform failed: %s", ex.what());
      return false;
    }
    out.x = static_cast<float>(out_stamped.point.x);
    out.y = static_cast<float>(out_stamped.point.y);
    out.z = static_cast<float>(out_stamped.point.z);
    return true;
  };

  // 创建锥桶 markers（圆柱体）
  for (size_t i = 0; i < msg->points.size(); ++i) {
    geometry_msgs::Point32 point_world;
    if (!transform_point(msg->points[i], point_world)) {
      continue;
    }
    float confidence = 1.0f;
    if (use_confidence_alpha_ && i < msg->confidence.size()) {
      confidence = msg->confidence[i];
    }

    int color_type = kConeNone;
    if (i < msg->color_types.size()) {
      color_type = normalizeConeType(static_cast<int>(msg->color_types[i]));
    } else if (!msg->color.data.empty()) {
      // 兼容旧消息：只有一个全局 color 字段
      color_type = parseLegacyColorChar(msg->color.data[0]);
    }

    auto marker = createConeMarker(point_world.x, point_world.y, point_world.z, static_cast<int>(i),
                                   color_type, "cone_detections", marker_frame, confidence);
    marker.header.stamp = msg->header.stamp;
    markers.markers.push_back(marker);
  }

  // 创建边界框 markers
  if (show_bounding_box_ && msg->minPoints.size() == msg->maxPoints.size()) {
    for (size_t i = 0; i < msg->minPoints.size(); ++i) {
      geometry_msgs::Point32 min_world, max_world;
      if (!transform_point(msg->minPoints[i], min_world) ||
          !transform_point(msg->maxPoints[i], max_world)) {
        continue;
      }
      auto bbox = createBoundingBoxMarker(min_world, max_world, static_cast<int>(i), "cone_bbox",
                                          marker_frame);
      bbox.header.stamp = msg->header.stamp;
      markers.markers.push_back(bbox);
    }
  }

  // 创建距离标签 markers
  if (show_distance_label_ && msg->obj_dist.size() == msg->points.size()) {
    for (size_t i = 0; i < msg->points.size(); ++i) {
      geometry_msgs::Point32 point_world;
      if (!transform_point(msg->points[i], point_world)) {
        continue;
      }
      auto label =
          createDistanceLabel(point_world.x, point_world.y, point_world.z, msg->obj_dist[i],
                              static_cast<int>(i), "cone_distance", marker_frame);
      label.header.stamp = msg->header.stamp;
      markers.markers.push_back(label);
    }
  }

  pub_markers_.publish(markers);
}

void ConeVisualizer::fusedConeDetectionsCallback(
    const autodrive_msgs::HUAT_FusedConeDetections::ConstPtr& msg) {
  if (!publish_detection_markers_ || !prefer_fused_detections_) {
    return;
  }

  visualization_msgs::MarkerArray markers;
  const std::string source_frame =
      !msg->lidar_frame.empty() ? msg->lidar_frame
                                : (msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id);
  const bool need_transform = source_frame != frame_id_;
  std::string marker_frame = source_frame;
  geometry_msgs::TransformStamped tf_msg;

  if (need_transform) {
    try {
      tf_msg = tf_buffer_.lookupTransform(frame_id_, source_frame, msg->header.stamp,
                                          ros::Duration(tf_timeout_sec_));
    } catch (const tf2::TransformException& ex) {
      if (std::string(ex.what()).find("extrapolation") != std::string::npos) {
        ++tf_extrapolation_failure_count_;
      }
      try {
        tf_msg = tf_buffer_.lookupTransform(frame_id_, source_frame, ros::Time(0),
                                            ros::Duration(tf_timeout_sec_));
      } catch (const tf2::TransformException& ex2) {
        ++tf_lookup_failure_count_;
        last_tf_failure_time_ = ros::Time::now();
        ROS_WARN_THROTTLE(1.0, "[ConeVisualizer] TF %s->%s unavailable for fused detections: %s",
                          source_frame.c_str(), frame_id_.c_str(), ex2.what());
        return;
      }
    }
    marker_frame = frame_id_;
  }

  auto transform_point = [&](const geometry_msgs::Point32& in,
                             geometry_msgs::Point32& out) -> bool {
    if (!need_transform) {
      out = in;
      return true;
    }
    geometry_msgs::PointStamped in_stamped;
    geometry_msgs::PointStamped out_stamped;
    in_stamped.header.frame_id = source_frame;
    in_stamped.header.stamp = msg->header.stamp;
    in_stamped.point.x = in.x;
    in_stamped.point.y = in.y;
    in_stamped.point.z = in.z;
    try {
      tf2::doTransform(in_stamped, out_stamped, tf_msg);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "[ConeVisualizer] Fused point transform failed: %s", ex.what());
      return false;
    }
    out.x = static_cast<float>(out_stamped.point.x);
    out.y = static_cast<float>(out_stamped.point.y);
    out.z = static_cast<float>(out_stamped.point.z);
    return true;
  };

  std::size_t matched_count = 0;
  std::size_t unmatched_count = 0;
  for (size_t i = 0; i < msg->points.size(); ++i) {
    geometry_msgs::Point32 point_world;
    if (!transform_point(msg->points[i], point_world)) {
      continue;
    }

    float confidence = 1.0f;
    if (use_confidence_alpha_ && i < msg->lidar_confidences.size()) {
      confidence = msg->lidar_confidences[i];
    }

    int color_type = kConeNone;
    if (i < msg->fused_color_types.size()) {
      color_type = normalizeConeType(static_cast<int>(msg->fused_color_types[i]));
    } else if (i < msg->lidar_color_types.size()) {
      color_type = normalizeConeType(static_cast<int>(msg->lidar_color_types[i]));
    }

    const std::uint8_t status = i < msg->association_status.size() ? msg->association_status[i] : 1;
    const bool matched = (status == kFusionMatched || status == kFusionLegacyHfovMatched);
    if (matched) {
      ++matched_count;
    } else {
      ++unmatched_count;
    }

    auto marker = createConeMarker(point_world.x, point_world.y, point_world.z, static_cast<int>(i),
                                   color_type, "cone_detections", marker_frame, confidence,
                                   unmatched_as_white_ && !matched);
    marker.header.stamp = msg->header.stamp;
    markers.markers.push_back(marker);
  }

  if (show_distance_label_ && msg->obj_dist.size() == msg->points.size()) {
    for (size_t i = 0; i < msg->points.size(); ++i) {
      geometry_msgs::Point32 point_world;
      if (!transform_point(msg->points[i], point_world)) {
        continue;
      }
      auto label =
          createDistanceLabel(point_world.x, point_world.y, point_world.z, msg->obj_dist[i],
                              static_cast<int>(i), "cone_distance", marker_frame);
      label.header.stamp = msg->header.stamp;
      markers.markers.push_back(label);
    }
  }

  last_fused_msg_time_ = ros::Time::now();
  last_detection_source_ = "fused";
  last_fused_matched_count_ = static_cast<int>(matched_count);
  last_fused_unmatched_count_ = static_cast<int>(unmatched_count);
  pub_markers_.publish(markers);
}

void ConeVisualizer::coneMapCallback(const autodrive_msgs::HUAT_ConeMap::ConstPtr& msg) {
  visualization_msgs::MarkerArray markers;

  // 创建锥桶 markers - 使用 cone 数组和 position_global
  for (size_t i = 0; i < msg->cone.size(); ++i) {
    const auto& cone = msg->cone[i];
    float confidence = 1.0f;
    if (use_confidence_alpha_ && cone.confidence > 0) {
      // uint32 归一化为 0~1（假设 confidence 范围 0~100）
      confidence = std::min(1.0f, static_cast<float>(cone.confidence) / 100.0f);
    }
    // Use global frame_id_ for global map
    auto marker =
        createConeMarker(cone.position_global.x, cone.position_global.y, cone.position_global.z,
                         static_cast<int>(cone.id), normalizeConeType(static_cast<int>(cone.type)),
                         "cone_map", frame_id_, confidence);
    marker.header.stamp = msg->header.stamp;
    markers.markers.push_back(marker);
  }

  pub_markers_.publish(markers);
}

visualization_msgs::Marker ConeVisualizer::createConeMarker(double x, double y, double z, int id,
                                                            int type, const std::string& ns,
                                                            const std::string& frame_id,
                                                            float confidence, bool force_white) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::Marker::ADD;

  if (use_mesh_) {
    // 3D DAE 网格模型
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    if (cone_mesh_type_ == "gazebo") {
      // Gazebo construction_cone：带纹理贴图，所有锥桶用同一模型
      marker.mesh_resource = MESH_CONE_GAZEBO;
      marker.mesh_use_embedded_materials = true;
      // DAE 单位 inch，scale=10 约 0.3m；大黄色锥桶放大以区分大小
      double mesh_scale = CONE_GAZEBO_SCALE;
      if (type == static_cast<int>(ConeType::YELLOW_BIG)) {
        mesh_scale *= 1.6;
      }
      marker.scale.x = mesh_scale;
      marker.scale.y = mesh_scale;
      marker.scale.z = mesh_scale;
    } else {
      // FSSIM 分色 DAE：蓝/黄/橙各一个模型，颜色内嵌
      marker.mesh_resource = getConeMeshURI(type);
      marker.mesh_use_embedded_materials = true;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
    }

    // DAE 原点在底部，不需要 z 偏移
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
  } else {
    // 原始 CYLINDER 渲染
    marker.type = visualization_msgs::Marker::CYLINDER;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z + cone_height_ / 2.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = cone_radius_ * 2.0;
    marker.scale.y = cone_radius_ * 2.0;
    marker.scale.z = cone_height_;
    if (type == static_cast<int>(ConeType::YELLOW_BIG)) {
      marker.scale.x *= 1.3;
      marker.scale.y *= 1.3;
      marker.scale.z *= 1.6;
      marker.pose.position.z = z + marker.scale.z / 2.0;
    }
  }

  // 颜色
  if (force_white || (use_mesh_ && cone_mesh_type_ == "gazebo")) {
    // Gazebo 纹理模型或显式未匹配高亮：统一白色
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
  } else {
    auto color = getConeColor(type);
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
  }

  // 置信度透明度映射
  if (use_confidence_alpha_) {
    marker.color.a = std::max(min_alpha_, confidence);
  } else {
    marker.color.a = 1.0f;
  }

  // Keep global map cones persistent (FSSIM-like behavior), while detections remain transient.
  if (ns == "cone_map") {
    marker.lifetime = ros::Duration(0.0);
  } else {
    marker.lifetime = ros::Duration(1.0);
  }

  return marker;
}

visualization_msgs::Marker ConeVisualizer::createBoundingBoxMarker(
    const geometry_msgs::Point32& min_pt, const geometry_msgs::Point32& max_pt, int id,
    const std::string& ns, const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.02;  // 线宽
  marker.color.r = cone_bbox_color_[0];
  marker.color.g = cone_bbox_color_[1];
  marker.color.b = cone_bbox_color_[2];
  marker.color.a = cone_bbox_color_[3];
  marker.lifetime = ros::Duration(0.2);

  // 构建立方体的12条边
  geometry_msgs::Point p;

  // 底面4条边
  p.x = min_pt.x;
  p.y = min_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);
  p.x = max_pt.x;
  p.y = min_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);

  p.x = max_pt.x;
  p.y = min_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);
  p.x = max_pt.x;
  p.y = max_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);

  p.x = max_pt.x;
  p.y = max_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);
  p.x = min_pt.x;
  p.y = max_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);

  p.x = min_pt.x;
  p.y = max_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);
  p.x = min_pt.x;
  p.y = min_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);

  // 顶面4条边
  p.x = min_pt.x;
  p.y = min_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);
  p.x = max_pt.x;
  p.y = min_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);

  p.x = max_pt.x;
  p.y = min_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);
  p.x = max_pt.x;
  p.y = max_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);

  p.x = max_pt.x;
  p.y = max_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);
  p.x = min_pt.x;
  p.y = max_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);

  p.x = min_pt.x;
  p.y = max_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);
  p.x = min_pt.x;
  p.y = min_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);

  // 4条竖边
  p.x = min_pt.x;
  p.y = min_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);
  p.x = min_pt.x;
  p.y = min_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);

  p.x = max_pt.x;
  p.y = min_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);
  p.x = max_pt.x;
  p.y = min_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);

  p.x = max_pt.x;
  p.y = max_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);
  p.x = max_pt.x;
  p.y = max_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);

  p.x = min_pt.x;
  p.y = max_pt.y;
  p.z = min_pt.z;
  marker.points.push_back(p);
  p.x = min_pt.x;
  p.y = max_pt.y;
  p.z = max_pt.z;
  marker.points.push_back(p);

  return marker;
}

visualization_msgs::Marker ConeVisualizer::createDistanceLabel(double x, double y, double z,
                                                               float distance, int id,
                                                               const std::string& ns,
                                                               const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z + 0.5;  // 在锥桶上方0.5m
  marker.pose.orientation.w = 1.0;

  marker.scale.z = 0.3;  // 文字高度

  marker.color.r = cone_label_color_[0];
  marker.color.g = cone_label_color_[1];
  marker.color.b = cone_label_color_[2];
  marker.color.a = cone_label_color_[3];

  char text[32];
  snprintf(text, sizeof(text), "%.1fm", distance);
  marker.text = text;

  marker.lifetime = ros::Duration(0.2);

  return marker;
}

// B8: TF timeout monitoring diagnostics
void ConeVisualizer::publishDiagnostics() {
  using DH = fsd_common::DiagnosticsHelper;

  uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string message = "OK";

  // Check for recent TF failures
  if (last_tf_failure_time_.isValid()) {
    const double time_since_failure = (ros::Time::now() - last_tf_failure_time_).toSec();
    if (time_since_failure < 5.0) {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      message = "TF_LOOKUP_FAILURES";
    }
  }

  std::vector<diagnostic_msgs::KeyValue> kvs;
  kvs.push_back(DH::KV("tf_timeout_sec", std::to_string(tf_timeout_sec_)));
  kvs.push_back(DH::KV("tf_lookup_failure_count", std::to_string(tf_lookup_failure_count_)));
  kvs.push_back(
      DH::KV("tf_extrapolation_failure_count", std::to_string(tf_extrapolation_failure_count_)));
  kvs.push_back(DH::KV("target_frame", frame_id_));
  kvs.push_back(DH::KV("prefer_fused_detections", prefer_fused_detections_ ? "1" : "0"));
  kvs.push_back(DH::KV("unmatched_as_white", unmatched_as_white_ ? "1" : "0"));
  kvs.push_back(DH::KV("last_detection_source", last_detection_source_));
  kvs.push_back(DH::KV("last_fused_matched_count", std::to_string(last_fused_matched_count_)));
  kvs.push_back(DH::KV("last_fused_unmatched_count", std::to_string(last_fused_unmatched_count_)));
  if (last_fused_msg_time_.isValid()) {
    kvs.push_back(
        DH::KV("fused_age_sec", std::to_string((ros::Time::now() - last_fused_msg_time_).toSec())));
  }

  diag_helper_.PublishStatus("cone_visualizer", "fsd_visualization", level, message, kvs,
                             ros::Time::now(), false);
}

}  // namespace fsd_viz
