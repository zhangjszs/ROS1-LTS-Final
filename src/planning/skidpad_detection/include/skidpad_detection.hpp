#ifndef FSAC_SKIDPAD_DETECTION_HPP_
#define FSAC_SKIDPAD_DETECTION_HPP_

#include <math.h>
#include <cmath>
#include <mutex>
#include <thread>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_CarState.h>
#include <set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>

struct comp
{
	bool operator()(const pcl::PointXYZ& a, const pcl::PointXYZ& b) const
	{
		if (a.x < b.x)
			return true;
		else if (a.x > b.x)
			return false;
		else if (a.y < b.y)
			return true;
		else if (a.y > b.y)
			return false;
		else if (a.z < b.z)
			return true;
		else
			return false;
	}
};
struct Trajectory
{
	double x;
	double y;
	double yaw;
	double v;
};
namespace fsac
{

	class Skidpad_detection
	{
	public:
		Skidpad_detection(ros::NodeHandle &nh);
		bool ChangPathFlag(double current_x, double current_y, double TargetX_, double TargetY_, double DistanceThreshold_);
		void runAlgorithm();
		void ChangLeavePathFlag(double current_x, double current_y, double TargetX_, double TargetY_, double LeaveDistanceThreshold_);
		void isApproaching(double current_x, double current_y, double FinTargetX_, double FInTargetY_, double stopdistance_);
		void publishTransformedPath(const nav_msgs::Path &path);
		void SavePosition();
		void SaveLinePath();
		void SaveRightPath();
		void SaveLeftPath();
		void SaveLastPath();
		double lipu;

	private:
		void PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_ptr);
		Trajectory current_pose;
		nav_msgs::Path ros_path_;
		std_msgs::Bool msg;
		geometry_msgs::PoseStamped pose;
		// geometry_msgs::Point path;
		pcl::PointCloud<pcl::PointXYZ>::Ptr skidpad_msg_ptr;
		std::set<pcl::PointXYZ, comp> orderCloud;
		std::set<pcl::PointXYZ, comp> orderCloud_real;
		pcl::PointXYZ pc_trans;
		ros::NodeHandle nh_;
		ros::Subscriber subscriber_;
		ros::Subscriber pose_sub_;
		ros::Subscriber path_sub_;
		ros::Publisher logging_path;
		ros::Publisher approachingGoalPub;
		std::string subTopic_;
		int inverse_flag;
		int modeFlag = 0;
		double circle2lidar_;
		double at2_angle_mid;
		double prev_at2_angle_mid;
		double targetX_, targetY_;
		double FinTargetX_, FInTargetY_;
		double distanceThreshold_, LeavedistanceThreshold_, stopdistance_;
		bool at2_angle_calced = false ; 
		bool find_four_bucket, matchFlag, changFlag, leaveFlag;
		bool haschanged = true;
		bool skipIteration = false;
		double mid_x_fir, mid_y_fir, mid_x_sec, mid_y_sec;

	private:
		void skidpadCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &skidpad_msg);
		void positionback(const autodrive_msgs::HUAT_CarState::ConstPtr &carposition);
		void loadParameters();
	};
}
#endif
