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
  // WayComputer::Speed
  pnh.param<double>("speed_cap_safe", wayComputer.speed.speed_cap_safe, 11.11);
  pnh.param<double>("speed_cap_fast", wayComputer.speed.speed_cap_fast, 13.33);
  pnh.param<double>("max_lateral_acc", wayComputer.speed.max_lateral_acc, 6.5);
  pnh.param<double>("max_accel", wayComputer.speed.max_accel, 2.5);
  pnh.param<double>("max_brake", wayComputer.speed.max_brake, 4.0);
  pnh.param<double>("min_speed", wayComputer.speed.min_speed, 1.0);
  pnh.param<double>("curvature_epsilon", wayComputer.speed.curvature_epsilon, 1e-3);
  pnh.param<int>("mode_transition_frames", wayComputer.speed.mode_transition_frames, 40);
  pnh.param<double>("curvature_limit", wayComputer.speed.curvature_limit, 0.222);
  pnh.param<double>("curvature_warn_limit_scale", wayComputer.speed.curvature_warn_limit_scale, 1.08);
  pnh.param<bool>("curvature_smoothing_enable", wayComputer.speed.curvature_smoothing_enable, true);
  pnh.param<int>("curvature_smoothing_max_iters", wayComputer.speed.curvature_smoothing_max_iters, 4);
  pnh.param<double>("curvature_smoothing_alpha", wayComputer.speed.curvature_smoothing_alpha, 0.25);
  pnh.param<double>("curvature_warn_ratio", wayComputer.speed.curvature_warn_ratio, 0.12);
  pnh.param<int>("curvature_warn_min_points", wayComputer.speed.curvature_warn_min_points, 8);
  pnh.param<int>("min_publish_path_size", wayComputer.speed.min_publish_path_size, 12);
  pnh.param<int>("short_path_stable_relax_points", wayComputer.speed.short_path_stable_relax_points, 3);
  pnh.param<bool>("hold_last_valid_path", wayComputer.speed.hold_last_valid_path, true);
  pnh.param<int>("hold_last_valid_max_frames", wayComputer.speed.hold_last_valid_max_frames, 25);
  pnh.param<int>("loop_close_debounce_frames", wayComputer.speed.loop_close_debounce_frames, 5);
  pnh.param<int>("loop_open_debounce_frames", wayComputer.speed.loop_open_debounce_frames, 15);
  pnh.param<bool>("loop_allow_reopen", wayComputer.speed.loop_allow_reopen, false);
  pnh.param<int>("mode_min_hold_frames", wayComputer.speed.mode_min_hold_frames, 30);
  pnh.param<bool>("search_adaptive_enable", wayComputer.speed.search_adaptive_enable, true);
  pnh.param<int>("search_adaptive_trigger_frames", wayComputer.speed.search_adaptive_trigger_frames, 1);
  pnh.param<int>("search_adaptive_step_frames", wayComputer.speed.search_adaptive_step_frames, 1);
  pnh.param<int>("search_adaptive_max_level", wayComputer.speed.search_adaptive_max_level, 8);
  pnh.param<int>("search_adaptive_bootstrap_level", wayComputer.speed.search_adaptive_bootstrap_level, 4);
  pnh.param<double>("search_adaptive_radius_step", wayComputer.speed.search_adaptive_radius_step, 3.0);
  pnh.param<double>("search_adaptive_heuristic_step", wayComputer.speed.search_adaptive_heuristic_step, 1.0);
  pnh.param<double>("search_adaptive_radius_max", wayComputer.speed.search_adaptive_radius_max, 40.0);
  pnh.param<double>("search_adaptive_heuristic_max", wayComputer.speed.search_adaptive_heuristic_max, 14.0);
  pnh.param<bool>("search_adaptive_retry_on_short", wayComputer.speed.search_adaptive_retry_on_short, true);
  pnh.param<bool>("loop_diag_enable", wayComputer.speed.loop_diag_enable, false);

  // Visualization
  pnh.param<bool>("publish_viz_data", visualization.publish_viz_data, true);
  pnh.param<std::string>("viz_topic", visualization.viz_topic, "planning/high_speed_tracking/viz");
}
