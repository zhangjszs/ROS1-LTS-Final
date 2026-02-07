/**
 * @file Param.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Param class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "utils/Params.hpp"

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

Params::Params(ros::NodeHandle *const nh) {
  (void)nh;
  ros::NodeHandle pnh("~");
  // Main
  main.package_path = ros::package::getPath("planning_ros");
  pnh.param<std::string>("input_cones_topic", main.input_cones_topic, "localization/cone_map");
  pnh.param<std::string>("input_pose_topic", main.input_pose_topic, "localization/car_state");
  pnh.param<std::string>("output_full_topic", main.output_full_topic, "planning/high_speed_tracking/pathlimits/full");
  pnh.param<std::string>("output_partial_topic", main.output_partial_topic, "planning/high_speed_tracking/pathlimits/partial");
  pnh.param<std::string>("stop_topic", main.stop_topic, "planning/high_speed_tracking/stop");
  pnh.param<bool>("shutdown_on_loop_closure", main.shutdown_on_loop_closure, true);
  pnh.param<bool>("debug_save_way_files", main.debug_save_way_files, false);
  pnh.param<float>("min_cone_confidence", main.min_cone_confidence, 0.0);
  pnh.param<int>("number_of_stopped_turns", main.number_of_stopped_turns, 3);
  pnh.param<int>("the_mode_of_partial_path", main.the_mode_of_partial_path, 2);
  pnh.param<int>("the_mode_of_full_path", main.the_mode_of_full_path, 3);

  // WayComputer
  pnh.param<double>("max_triangle_edge_len", wayComputer.max_triangle_edge_len, 9.0);
  pnh.param<double>("min_triangle_angle", wayComputer.min_triangle_angle, 0.25);
  pnh.param<double>("max_dist_circum_midPoint", wayComputer.max_dist_circum_midPoint, 1.0);
  pnh.param<int>("failsafe_max_way_horizon_size", wayComputer.failsafe_max_way_horizon_size, 6);
  pnh.param<bool>("general_failsafe", wayComputer.general_failsafe, true);
  pnh.param<double>("general_failsafe_safetyFactor", wayComputer.general_failsafe_safetyFactor, 1.4);
  // WayComputer::Search
  pnh.param<int>("max_way_horizon_size", wayComputer.search.max_way_horizon_size, 0);
  pnh.param<int>("max_search_tree_height", wayComputer.search.max_search_tree_height, 5);
  pnh.param<double>("search_radius", wayComputer.search.search_radius, 5.0);
  pnh.param<double>("max_angle_diff", wayComputer.search.max_angle_diff, 0.6);
  pnh.param<double>("edge_len_diff_factor", wayComputer.search.edge_len_diff_factor, 0.5);
  pnh.param<int>("max_search_options", wayComputer.search.max_search_options, 2);
  pnh.param<double>("max_next_heuristic", wayComputer.search.max_next_heuristic, 3.0);
  pnh.param<float>("heur_dist_ponderation", wayComputer.search.heur_dist_ponderation, 0.6);
  pnh.param<bool>("allow_intersection", wayComputer.search.allow_intersection, false);
  pnh.param<float>("max_treeSearch_time", wayComputer.search.max_treeSearch_time, 0.05);
  // WayComputer::Way
  pnh.param<double>("max_dist_loop_closure", wayComputer.way.max_dist_loop_closure, 1.0);
  pnh.param<double>("max_angle_diff_loop_closure", wayComputer.way.max_angle_diff_loop_closure, 0.6);
  pnh.param<int>("vital_num_midpoints", wayComputer.way.vital_num_midpoints, 5);

  // Visualization
  pnh.param<bool>("publish_viz_data", visualization.publish_viz_data, true);
  pnh.param<std::string>("viz_topic", visualization.viz_topic, "planning/high_speed_tracking/viz");
}
