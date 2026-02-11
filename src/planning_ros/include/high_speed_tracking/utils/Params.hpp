/**
 * @file Params.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Params class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <ros/package.h>
#include <ros/ros.h>

/**
 * @brief Represents all the parameters that the program needs divided into
 * modules and structures.
 * The class pretends to be a bridge between the .yaml and the program modules
 * and structures.
 * When constructed, it fills all parameters using ROS param.
 */
class Params {
 public:
  Params(ros::NodeHandle *const nh);
  struct Main {
    std::string input_cones_topic, input_pose_topic, stop_topic;
    std::string markers_full_topic, markers_partial_topic;
    std::string package_path;
    bool shutdown_on_loop_closure;
    bool debug_save_way_files;
    float min_cone_confidence;
    int number_of_stopped_turns;
    int the_mode_of_partial_path,the_mode_of_full_path;
  } main;
  struct WayComputer {
    double max_triangle_edge_len, min_triangle_angle, max_dist_circum_midPoint;
    int failsafe_max_way_horizon_size;
    bool general_failsafe;
    double general_failsafe_safetyFactor;
    struct Search {
      int max_way_horizon_size;
      int max_search_tree_height;
      double search_radius, max_angle_diff, edge_len_diff_factor;
      int max_search_options;
      double max_next_heuristic;
      float heur_dist_ponderation;
      bool allow_intersection;
      float max_treeSearch_time;
    } search;
    struct Way {
      double max_dist_loop_closure;
      double max_angle_diff_loop_closure;
      int vital_num_midpoints;
    } way;
    struct Speed {
      double speed_cap_safe;
      double speed_cap_fast;
      double max_lateral_acc;
      double max_accel;
      double max_brake;
      double min_speed;
      double curvature_epsilon;
      int mode_transition_frames;
      double curvature_limit;
      double curvature_warn_limit_scale;
      bool curvature_smoothing_enable;
      int curvature_smoothing_max_iters;
      double curvature_smoothing_alpha;
      double curvature_warn_ratio;
      int curvature_warn_min_points;
      int min_publish_path_size;
      int short_path_stable_relax_points;
      bool hold_last_valid_path;
      int hold_last_valid_max_frames;
      int loop_close_debounce_frames;
      int loop_open_debounce_frames;
      bool loop_allow_reopen;
      int mode_min_hold_frames;
      bool search_adaptive_enable;
      int search_adaptive_trigger_frames;
      int search_adaptive_step_frames;
      int search_adaptive_max_level;
      int search_adaptive_bootstrap_level;
      double search_adaptive_radius_step;
      double search_adaptive_heuristic_step;
      double search_adaptive_radius_max;
      double search_adaptive_heuristic_max;
      bool search_adaptive_retry_on_short;
      bool loop_diag_enable;
    } speed;
  } wayComputer;
  struct Visualization {
    bool publish_viz_data;
    std::string viz_topic;
  } visualization;
};
