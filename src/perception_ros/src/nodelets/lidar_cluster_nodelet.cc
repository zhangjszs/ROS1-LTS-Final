
#include <string>

#include <boost/thread.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <perception_ros/lidar_cluster_ros.hpp>

namespace nodelet_lidar
{
  class Lidar_Cluster_Nodelet : public nodelet::Nodelet
  {
  public:
    Lidar_Cluster_Nodelet() : running_(false)
    {
    }

    ~Lidar_Cluster_Nodelet()
    {
      if (running_)
      {
        NODELET_INFO("shutting down lidar_cluster algoPoll thread");
        running_ = false;
        lidarThread_->join();
        NODELET_INFO("lidar_cluster algoPoll thread stopped");
      }
    }

  private:
    virtual void onInit(void);
    virtual void algoPoll(void);

    volatile bool running_; ///< device thread is running
    boost::shared_ptr<boost::thread> lidarThread_;
    boost::shared_ptr<boost::thread> imuThread_;
    boost::shared_ptr<perception_ros::LidarClusterRos> lc_; ///< driver implementation class
  };

  void Lidar_Cluster_Nodelet::onInit()
  {
    // start the driver
    lc_.reset(new perception_ros::LidarClusterRos(getNodeHandle(), getPrivateNodeHandle()));
    running_ = true;
    lidarThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Lidar_Cluster_Nodelet::algoPoll, this)));
  }

  /** @brief Device poll thread main loop. */
  void Lidar_Cluster_Nodelet::algoPoll()
  {
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      // poll device until end of file
      ros::spinOnce();
      lc_->RunOnce();
      loop_rate.sleep();
      // controls its process freq
    }
    running_ = false;
  }

} // namespace velodyne_driver

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: class type, base class type

PLUGINLIB_EXPORT_CLASS(nodelet_lidar::Lidar_Cluster_Nodelet, nodelet::Nodelet)
