/**
 * @file Visualization.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Visualization class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "modules/Visualization.hpp"
#include <autodrive_msgs/topic_contract.hpp>

/* ----------------------------- Private Methods ---------------------------- */

void Visualization::setTimestamp(const ros::Time &stamp) {
  this->stamp_ = stamp;
  viz_msg_.header.stamp = stamp;
  viz_msg_.header.frame_id = autodrive_msgs::frame_contract::kWorld;  // 全局坐标系
}

/* ------------------------------ Public Methods ---------------------------- */

Visualization &Visualization::getInstance() {
  static Visualization vis;
  return vis;
}

void Visualization::init(ros::NodeHandle *const nh, const Params::Visualization &params) {
  params_ = params;
  if (params.publish_viz_data) {
    viz_pub_ = nh->advertise<autodrive_msgs::HUAT_HighSpeedViz>(params_.viz_topic, 1, true);
  }
}

void Visualization::visualize(const TriangleSet &triSet) {
  if (not this->params_.publish_viz_data) return;

  viz_msg_.triangulation_lines.clear();
  viz_msg_.circumcenters.clear();
  viz_msg_.triangle_edge_midpoints.clear();

  viz_msg_.triangulation_lines.reserve(triSet.size() * 6);
  viz_msg_.circumcenters.reserve(triSet.size());
  viz_msg_.triangle_edge_midpoints.reserve(triSet.size() * 3);

  auto push_line = [&](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
    viz_msg_.triangulation_lines.push_back(a);
    viz_msg_.triangulation_lines.push_back(b);
  };

  for (const Triangle &t : triSet) {
    const geometry_msgs::Point p0 = t.nodes[0].pointGlobal().gmPoint();
    const geometry_msgs::Point p1 = t.nodes[1].pointGlobal().gmPoint();
    const geometry_msgs::Point p2 = t.nodes[2].pointGlobal().gmPoint();

    push_line(p0, p1);
    push_line(p1, p2);
    push_line(p2, p0);

    viz_msg_.circumcenters.push_back(t.circumCenterGlobal().gmPoint());

    for (const Edge &e : t.edges) {
      viz_msg_.triangle_edge_midpoints.push_back(e.midPointGlobal().gmPoint());
    }
  }
}

void Visualization::visualize(const EdgeSet &edgeSet) {
  if (not this->params_.publish_viz_data) return;

  viz_msg_.edge_midpoints.clear();
  viz_msg_.edge_midpoints.reserve(edgeSet.size());
  for (const Edge &e : edgeSet) {
    viz_msg_.edge_midpoints.push_back(e.midPointGlobal().gmPoint());
  }
}

void Visualization::visualize(const Way &way) {
  if (not this->params_.publish_viz_data) return;

  viz_msg_.path.clear();
  viz_msg_.left.clear();
  viz_msg_.right.clear();

  for (const Point &p : way.getPath()) {
    viz_msg_.path.push_back(p.gmPoint());
  }

  Tracklimits tracklimits = way.getTracklimits();
  for (const Node &n : tracklimits.first) {
    viz_msg_.left.push_back(n.pointGlobal().gmPoint());
  }
  for (const Node &n : tracklimits.second) {
    viz_msg_.right.push_back(n.pointGlobal().gmPoint());
  }

  if (viz_pub_.getNumSubscribers() > 0) {
    viz_pub_.publish(viz_msg_);
  }
}
