
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
      if (running_ && lidarThread_)
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
    if (lc_->IsLegacyPollMode())
    {
      running_ = true;
      lidarThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Lidar_Cluster_Nodelet::algoPoll, this)));
      NODELET_INFO("lidar_cluster running in legacy_poll mode");
    }
    else
    {
      running_ = false;
      NODELET_INFO("lidar_cluster running in event_driven mode");
    }
  }

  /** @brief Device poll thread main loop. */
  void Lidar_Cluster_Nodelet::algoPoll()
  {
    ros::Rate loop_rate(lc_->LegacyPollHz());
    while (running_ && ros::ok())
    {
      // legacy polling path: callback only updates latest frame, poll thread consumes it.
      lc_->RunOnce();
      loop_rate.sleep();
    }
    running_ = false;
  }

} // namespace nodelet_lidar

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: class type, base class type

PLUGINLIB_EXPORT_CLASS(nodelet_lidar::Lidar_Cluster_Nodelet, nodelet::Nodelet)
