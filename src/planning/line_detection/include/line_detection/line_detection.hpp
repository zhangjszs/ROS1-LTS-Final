#ifndef FSAC_LINE_DETECTION_HPP_
#define FSAC_LINE_DETECTION_HPP_

#include <math.h>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_CarState.h>
#include <Eigen/Dense>

namespace fsac
{

/**
 * @brief Hough line parameters
 */
struct HoughLine
{
  double rho;    // Distance from origin to line
  double theta;  // Angle from x-axis to normal
  int votes;     // Number of votes for this line
};

/**
 * @brief Cone position in vehicle coordinates
 */
struct ConePoint
{
  double x;
  double y;
  double z;

  ConePoint(double x_ = 0.0, double y_ = 0.0, double z_ = 0.0)
    : x(x_), y(y_), z(z_) {}
};

/**
 * @brief Straight line detection and path planning class
 *
 * This class implements the Hough transform algorithm to detect straight lines
 * from cone positions detected by the perception module. It plans a straight
 * path for the vehicle to follow in the acceleration event.
 */
class LineDetection
{
public:
  /**
   * @brief Constructor
   * @param nh ROS NodeHandle
   */
  LineDetection(ros::NodeHandle &nh);

  /**
   * @brief Run the line detection and path planning algorithm
   */
  void runAlgorithm();

  /**
   * @brief Check if the vehicle has reached the finish line
   * @return true if finished, false otherwise
   */
  bool isFinished() const { return finished_; }

  /**
   * @brief Get the planned path
   * @return The planned path in vehicle coordinates
   */
  nav_msgs::Path getPlannedPath() const { return planned_path_; }

private:
  /**
   * @brief Callback for cone position messages from perception
   * @param cone_msg Cone position message
   */
  void coneCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &cone_msg);

  /**
   * @brief Callback for vehicle state messages
   * @param car_state Vehicle state message
   */
  void carStateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &car_state);

  /**
   * @brief Load parameters from parameter server
   */
  void loadParameters();

  /**
   * @brief Filter cones by distance and region of interest
   * @param cones All cones
   * @return Filtered cones
   */
  std::vector<ConePoint> filterCones(const std::vector<ConePoint> &cones);

  /**
   * @brief Detect lines using Hough transform
   * @param cones Filtered cone positions
   * @return Detected lines sorted by number of votes
   */
  std::vector<HoughLine> houghTransform(const std::vector<ConePoint> &cones);

  /**
   * @brief Select the best two lines (left and right boundaries)
   * @param lines All detected lines
   * @return Pair of (left_line, right_line)
   */
  std::pair<HoughLine, HoughLine> selectBoundaryLines(const std::vector<HoughLine> &lines);

  /**
   * @brief Calculate the center line from two boundary lines
   * @param left_line Left boundary line
   * @param right_line Right boundary line
   * @return Center line parameters
   */
  HoughLine calculateCenterLine(const HoughLine &left_line, const HoughLine &right_line);

  /**
   * @brief Generate path points along the center line
   * @param center_line Center line parameters
   * @return Path points in vehicle coordinates
   */
  std::vector<geometry_msgs::PoseStamped> generatePath(const HoughLine &center_line);

  /**
   * @brief Convert path from vehicle coordinates to world (ENU) coordinates
   * @param path Path in vehicle coordinates
   * @return Path in world coordinates
   */
  nav_msgs::Path convertToWorldCoordinates(const std::vector<geometry_msgs::PoseStamped> &path);

  /**
   * @brief Check if the vehicle has reached the finish line
   * @param current_x Vehicle x position in vehicle coordinates
   * @param finish_x Finish line x position
   * @return true if reached, false otherwise
   */
  bool checkFinishLine(double current_x, double finish_x);

  /**
   * @brief Publish the planned path
   */
  void publishPath();

  // ROS interface
  ros::NodeHandle nh_;
  ros::Subscriber cone_sub_;
  ros::Subscriber car_state_sub_;
  ros::Publisher path_pub_;
  ros::Publisher finish_pub_;

  // Parameters
  double hough_rho_resolution_;       // Hough transform rho resolution (meters)
  double hough_theta_resolution_;    // Hough transform theta resolution (radians)
  int hough_min_votes_;              // Minimum votes to consider a line valid
  double theta_tolerance_;           // Tolerance for theta angle (radians)
  double max_cone_distance_;         // Maximum cone distance to consider (meters)
  double min_cone_distance_;         // Minimum cone distance to consider (meters)
  double path_interval_;             // Interval between path points (meters)
  double max_path_distance_;         // Maximum path distance (meters)
  double imu_offset_x_;              // IMU to lidar offset in x (meters)
  double finish_line_threshold_;     // Distance threshold for finish line (meters)
  std::string cone_topic_;           // Cone position topic name
  std::string car_state_topic_;      // Car state topic name
  std::string path_topic_;           // Path output topic name

  // Algorithm state
  std::vector<ConePoint> cone_positions_;
  bool first_detection_done_;
  double finish_line_x_;             // Finish line x position in vehicle coordinates
  HoughLine center_line_;            // Detected center line
  bool finished_;

  // Vehicle state
  struct VehicleState
  {
    double x;      // Position in vehicle coordinates
    double y;      // Position in vehicle coordinates
    double theta;  // Yaw angle (radians)
    double v;      // Velocity (m/s)

    VehicleState() : x(0.0), y(0.0), theta(0.0), v(0.0) {}
  } vehicle_state_;

  // Planned path
  nav_msgs::Path planned_path_;
  std::mutex data_mutex_;
};

} // namespace fsac

#endif // FSAC_LINE_DETECTION_HPP_
