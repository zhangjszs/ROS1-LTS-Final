#include "skidpad_detection.hpp"
#include <ros/package.h>
namespace fsac
{

  Skidpad_detection::Skidpad_detection(ros::NodeHandle &nh) : nh_(nh)
  {
    loadParameters();
    matchFlag = true;
    subscriber_ = nh_.subscribe("/cone_position", 100000, &Skidpad_detection::skidpadCallback, this);
    pose_sub_ = nh_.subscribe("/Carstate", 1000, &Skidpad_detection::positionback, this);
    logging_path = nh_.advertise<nav_msgs::Path>("log_path", 10, true);
    approachingGoalPub = nh_.advertise<std_msgs::Bool>("approaching_goal", 10);
    skidpad_msg_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

  void Skidpad_detection::loadParameters()
  {
    ROS_INFO("skidpad_detection loading Parameters");
    if (!nh_.param("length/circle2lidar", circle2lidar_, 15.0))
    {
      ROS_WARN_STREAM("Did not load circle2lidar. Standard value is: " << circle2lidar_);
    }

    ROS_INFO("load parameters");
    if (!nh_.param("length/targetX", targetX_, 16.0))
    {
      ROS_WARN_STREAM("Did not load targetX. Standard value is: " << targetX_);
    }

    if (!nh_.param("length/targetY", targetY_, -1.0))
    {
      ROS_WARN_STREAM("Did not load targetY. Standard value is " << targetY_);
    }

    if (!nh_.param("length/FinTargetX", FinTargetX_, 40.0))
    {
      ROS_WARN_STREAM("Did not load FinTargetX. Standard value is: " << FinTargetX_);
    }

    if (!nh_.param("length/FinTargetY", FInTargetY_, 0.0))
    {
      ROS_WARN_STREAM("Did not load FinTargetY. Standard value is " << FInTargetY_);
    }

    if (!nh_.param("length/distanceThreshold", distanceThreshold_, 0.5))
    {
      ROS_WARN_STREAM("Did not load distanceThreshold. Standard value is " << distanceThreshold_);
    }

    if (!nh_.param("length/leavedistanceThreshold", LeavedistanceThreshold_, 1.0))
    {
      ROS_WARN_STREAM("Did not load distanceThreshold. Standard value is " << LeavedistanceThreshold_);
    }

    if (!nh_.param<std::string>("filtered_topic_name", subTopic_, "/skidpad_detection"))
    {
      ROS_WARN_STREAM("Did not load topic name. Standard value is: " << subTopic_);
    }

    if (!nh_.param("inverse_flag", inverse_flag, 1))
    {
      ROS_WARN_STREAM("Did not load topic name. Standard value is: " << inverse_flag);
    }

    if (!nh_.param("length/stopdistance", stopdistance_, 5.0))
    {
      ROS_WARN_STREAM("Did not load topic name. Standard value is: " << stopdistance_);
    }
    return;
  }

  void Skidpad_detection::PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_ptr)
  {
    // matchFlag = true;
    //  auto startTime = std::chrono::steady_clock::now();
    // PointCloudConstPtr cloud_ptr(in_ptr);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in_ptr);
    // 当线由车后延到车头这个方向，左右是y，前后是x，上下是z，右y是负的，左y是正的
    //  pass.filter(*cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.1, 15);
    pass.filter(*in_ptr);
    
    pass.setFilterFieldName("y"); // y:-1 represent right,+1 represent left
    pass.setFilterLimits(-3, 3);
    pass.filter(*in_ptr);
    // auto endTime = std::chrono::steady_clock::now();
    //   auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //   std::cout << "PassThrough took " << elapsedTime.count() << " milliseconds" << std::endl;
  }

  void Skidpad_detection::skidpadCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &skidpad_msg)
  {
    // ROS_WARN("d253a4d5a5da4d");
    skidpad_msg_ptr->clear();
    if (!matchFlag && !at2_angle_calced)
    {
      // order.clear();
      return;
    }
    for (const geometry_msgs::Point32 &point : skidpad_msg->points)
    {
      pcl::PointXYZ pcl_point;
      pcl_point.x = point.x;
      pcl_point.y = point.y;
      pcl_point.z = point.z;
      skidpad_msg_ptr->push_back(pcl_point);
      std::cout << "Point: x = " << point.x << ", y = " << point.y << ", z = " << point.z << std::endl;
    }
    PassThrough(skidpad_msg_ptr);
    orderCloud.clear();
    find_four_bucket = false;
    at2_angle_calced = false;
    if (matchFlag and !find_four_bucket) // matchFlag and !find_four_bucket
    {
      // std::cout << "right situation (matchFlag: " << matchFlag << ", find_four_bucket: " << find_four_bucket << ")" << std::endl;
      int obtain_four = 0;
      std::vector<pcl::PointXYZ> order;
      for (int i = 0; i < skidpad_msg_ptr->points.size(); i++)
      {
        pc_trans.x = skidpad_msg_ptr->points[i].x;
        pc_trans.y = skidpad_msg_ptr->points[i].y;
        pc_trans.z = skidpad_msg_ptr->points[i].z;
        orderCloud.insert(pc_trans);
      }
      std::vector<pcl::PointXYZ> ordered;
      std::set<pcl::PointXYZ>::iterator it = orderCloud.begin();
      for (; it != orderCloud.end(); it++)
      {
        if (obtain_four == 4)
        {
          std::cout << "find four " << std::endl;
          find_four_bucket = true;
          break;
        }
        order.push_back(*it);
        obtain_four++;
      }
      if (!find_four_bucket)
      {
        std::cout << "miss four -bucket！" << std::endl;
        return;
      }
      std::cout << "[0]:" << order[0].x << " " << order[0].y << std::endl;
      std::cout << "[1]:" << order[1].x << " " << order[1].y << std::endl;
      std::cout << "[2]:" << order[2].x << " " << order[2].y << std::endl;
      std::cout << "[3]:" << order[3].x << " " << order[3].y << std::endl;
      mid_x_fir = (order[0].x + order[1].x) / 2;
      mid_y_fir = (order[0].y + order[1].y) / 2;

      mid_x_sec = (order[2].x + order[3].x) / 2;
      mid_y_sec = (order[2].y + order[3].y) / 2;

      at2_angle_mid = atan2(std::abs(mid_y_sec - mid_y_fir), std::abs(mid_x_sec - mid_x_fir));
      lipu = at2_angle_mid;
      std::cout << "angle_mid:" << at2_angle_mid << std::endl;
      if (find_four_bucket)
        at2_angle_calced = true;
      return;
    }
    else
    {
      // ROS_INFO("matchFlag_callback");
      std::cout << "wrong situation (matchFlag: " << matchFlag << ", find_four_bucket: " << find_four_bucket << ")" << std::endl;
      return;
    }
  }

  void Skidpad_detection::positionback(const autodrive_msgs::HUAT_CarState::ConstPtr &carposition)
  {
    // 提取位置信息
    current_pose.x = carposition->car_state.x;
    current_pose.y = carposition->car_state.y;
    current_pose.yaw = carposition->car_state.theta;
    current_pose.v = carposition->V;
    // trajectoryContainer.push_back(current_pose);
    // saveposition(); //保存位置信息
    // 输出位置信息
    // std::cout << "车辆实时位置： x = " << current_pose.x << ", y = " << current_pose.y << ", yaw = " << current_pose.yaw << ", v = " << current_pose.v << std::endl;
  }

  bool Skidpad_detection::ChangPathFlag(double current_x, double current_y, double TargetX_, double TargetY_, double DistanceThreshold_)
  {
    // double distance = std::sqrt(std::pow((TargetX_ - current_x), 2) + std::pow((TargetY_ - current_y), 2));
    double dx = TargetX_ - current_x;
    double dy = TargetY_ - current_y;
    double distance = std::hypot(dx, dy);
    // std::cout << "distance= " << distance << " bool=" << (distance - DistanceThreshold_) << std::endl;
    return distance < DistanceThreshold_;
  }

  void Skidpad_detection::ChangLeavePathFlag(double current_x, double current_y, double TargetX_, double TargetY_, double LeaveDistanceThreshold_)
  {
    // double distance = std::sqrt(std::pow((TargetX_ - current_x), 2) + std::pow((TargetY_ - current_y), 2));
    double dx = TargetX_ - current_x;
    double dy = TargetY_ - current_y;
    double distance = std::hypot(dx, dy);
    if (distance >= LeaveDistanceThreshold_)
      haschanged = true;
    return;
  }

  void Skidpad_detection::isApproaching(double current_x, double current_y, double FinTargetX_, double FInTargetY_, double stopdistance_)
  {
    double dx = FinTargetX_ - current_x;
    double dy = FInTargetY_ - current_y;
    double distance = std::hypot(dx, dy);
    if (distance <= stopdistance_)
    {
      msg.data = 1; // 车辆接近终点
    }
    else
    {
      msg.data = 0; // 车辆距离终点较远
    }
    approachingGoalPub.publish(msg); // 发布消息
    // std::cout << "车辆是否接近终点：" << msg.data << std::endl;
  }

  void Skidpad_detection::publishTransformedPath(const nav_msgs::Path &path)
  {
    nav_msgs::Path transformed_path = path; // 复制路径信息

    // 进行路径点的坐标变换
    double angle = at2_angle_mid - lipu;                            // 计算变换的角度 - lipu
    Eigen::AngleAxisf t_V(angle, Eigen::Vector3f::UnitZ());         // 创建旋转变换对象
    Eigen::Matrix3f rotate_matrix = t_V.matrix();                   // 获取旋转矩阵
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity(); // 创建变换矩阵并初始化为单位矩阵

    if (inverse_flag)
      transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse(); // 将旋转矩阵的逆矩阵赋值给变换矩阵的左上角 3x3 子矩阵
    else
      transform_matrix.block<3, 3>(0, 0) = rotate_matrix; // 将旋转矩阵赋值给变换矩阵的左上角 3x3 子矩阵

    for (size_t i = 0; i < transformed_path.poses.size(); i++)
    {
      double temp_x = transformed_path.poses[i].pose.position.x; // 获取当前路径点的 x 坐标
      double temp_y = transformed_path.poses[i].pose.position.y; // 获取当前路径点的 y 坐标

      Eigen::Vector4f temp(temp_x, temp_y, 0, 1);       // 创建当前路径点的 4 维向量
      Eigen::Vector4f result = transform_matrix * temp; // 进行坐标变换

      transformed_path.poses[i].pose.position.x = result[0] / result[3]; // 更新路径点的 x 坐标
      transformed_path.poses[i].pose.position.y = result[1] / result[3]; // 更新路径点的 y 坐标
    }

    // 发布变换后的路径
    logging_path.publish(transformed_path);
  }

  void Skidpad_detection::runAlgorithm()
  {
    double interval = 0.05;
    double forward_distance = circle2lidar_;
    // double forward_distance = 13.0;
    double circle_radius = 9.125;   // original r = 9.125
    const double car_length = 1.87; // original l = 1.0
    double right_circle_x = forward_distance + car_length;
    double right_circle_y = -circle_radius;
    double left_circle_x = right_circle_x;
    double left_circle_y = circle_radius;
    // 这是三个判断的函数
    changFlag = ChangPathFlag(current_pose.x, current_pose.y, targetX_, targetY_, distanceThreshold_);
    ChangLeavePathFlag(current_pose.x, current_pose.y, targetX_, targetY_, LeavedistanceThreshold_);
    isApproaching(current_pose.x, current_pose.y, FinTargetX_, FInTargetY_, stopdistance_); // 这个判定是否接近终点
      std::cout << "runAlgorithm:" << matchFlag << "  " << at2_angle_calced << std::endl;
    // std::cout << "现在车的情况:"
    //           << "changFlag  " << changFlag << " "
    //           << "haschanged  " << haschanged << std::endl;
    // ROS_WARN_STREAM("matchFlag"<<matchFlag<<"at2_angle_calced")
    if (matchFlag and at2_angle_calced)
    {
      ros_path_.header.frame_id = "velodyne"; //
      // ros_path_.header.frame_id = "world";
      pose.header = ros_path_.header;
      ros_path_.poses.clear(); // 清空之前的路径信息
                               // 第一段路径：直线行驶部分
      modeFlag = 1;
      for (double i = 0.5; i < (car_length + forward_distance); i += interval) // 已惯导为坐标轴原点interval
      {
        pose.pose.position.x = i;
        pose.pose.position.y = 0;
        ros_path_.poses.push_back(pose);
      }
      // 储存第一段路径
      // void SaveLinePath();

      publishTransformedPath(ros_path_); // 发布第一段路径
      std::cout << "发布第一段路径 " << modeFlag << std::endl;
      modeFlag++;
      matchFlag = false;
      at2_angle_calced = false;
    }
    if (changFlag) //&& haschanged
    {
      if (haschanged)
      {
        switch (modeFlag)
        {
        case 4:
        case 5:
          // 第二段路径：右圆
          ros_path_.poses.clear();                              // 清空路径信息
          pose.pose.position.x = car_length + forward_distance; // 设置起点x坐标为第一段路径的终点x坐标
          pose.pose.position.y = 0;                             // 设置起点y坐标为0
          for (double i = 0; i < 2 * M_PI; i += interval / circle_radius)
          {
            pose.pose.position.x = circle_radius * std::cos(90 * M_PI / 180 - i) + right_circle_x;
            pose.pose.position.y = circle_radius * std::sin(90 * M_PI / 180 - i) + right_circle_y;
            ros_path_.poses.push_back(pose);
          }

          // 储存第二段路径
          // void SaveRightPath();

          publishTransformedPath(ros_path_); // 发布第二段路径
          std::cout << "发布第二段路径 " << modeFlag << std::endl;
          modeFlag++;
          haschanged = false;
          break;
        case 2:
        case 3:
          // 第三段路径：左圆
          ros_path_.poses.clear();                              // 清空路径信息
          pose.pose.position.x = car_length + forward_distance; // 设置起点x坐标为第一段路径的终点x坐标
          pose.pose.position.y = 0;                             // 设置起点y坐标为0
          for (double i = 0; i < 2 * M_PI; i += interval / circle_radius)
          {
            pose.pose.position.x = circle_radius * std::cos(-90 * M_PI / 180 + i) + left_circle_x;
            pose.pose.position.y = circle_radius * std::sin(-90 * M_PI / 180 + i) + left_circle_y;
            ros_path_.poses.push_back(pose);
          }

          // 储存第三段路径
          // void SaveLeftPath();

          publishTransformedPath(ros_path_); // 发布第三段路径
          std::cout << "发布第三段路径 " << modeFlag << std::endl;
          modeFlag++;
          haschanged = false;
          break;
        case 6:
          // 第四段路径：直线行驶部分
          ros_path_.poses.clear();                              // 清空路径信息
          pose.pose.position.x = car_length + forward_distance; // 设置起点x坐标为第一段路径的终点x坐标
          pose.pose.position.y = 0;                             // 设置起点y坐标为0
          for (float i = (car_length + forward_distance) + 2.0; i < (car_length + forward_distance) + 20.0; i += interval)
          {
            pose.pose.position.x = i;
            pose.pose.position.y = 0;
            ros_path_.poses.push_back(pose);
          }
          // 储存第四段路径
          // void SaveLastPath();
          publishTransformedPath(ros_path_); // 发布第四段路径
          std::cout << "发布第四段路径 " << modeFlag << std::endl;
          modeFlag++;
          haschanged = false;
          break;
        default:
          ROS_WARN("Invalid moduleFlag. No path to publish.");
          std::cout << modeFlag << std::endl;
          break;
        }
      }
    }
  }

  void Skidpad_detection::SavePosition()
  {
    std::ofstream outfile;
    const std::string file_path = ros::package::getPath("skidpad_detection") + "/src/trajectory.txt"; // 创建文件输出流对象
    outfile.open(file_path, std::ios::out | std::ios::app);
    if (!outfile)
    {
      std::cerr << "Failed to open the file: " << file_path << "\n";
      return;
    }

    outfile << "x:" << current_pose.x << " "
            << "y:" << current_pose.y << " "
            << "yaw:" << current_pose.yaw << " "
            << "v:" << current_pose.v << "\n";
    outfile.close();
  }

  void Skidpad_detection::SaveLinePath()
  {
    std::ofstream outfileL;
    const std::string file_path = ros::package::getPath("skidpad_detection") + "/src/LinePath.txt"; // 创建文件输出流对象
    outfileL.open(file_path, std::ios::out | std::ios::app);
    if (!outfileL)
    {
      std::cerr << "Failed to open the file: " << file_path << "\n";
      return;
    }
    for (const auto &pose : ros_path_.poses)
    {
      outfileL << "LinePath: "
               << pose.pose.position.x << " "
               << pose.pose.position.y << "\n";
    }
    outfileL.close();
  }

  void Skidpad_detection::SaveRightPath()
  {
    std::ofstream outfileR;
    const std::string file_path = ros::package::getPath("skidpad_detection") + "/src/RightPath.txt"; // 创建文件输出流对象
    outfileR.open(file_path, std::ios::out | std::ios::app);
    if (!outfileR)
    {
      std::cerr << "Failed to open the file: " << file_path << "\n";
      return;
    }
    for (const auto &pose : ros_path_.poses)
    {
      outfileR << "RightPath: "
               << pose.pose.position.x << " "
               << pose.pose.position.y << "\n";
    }
    outfileR.close();
  }

  void Skidpad_detection::SaveLeftPath()
  {
    std::ofstream outfileL;
    const std::string file_path = ros::package::getPath("skidpad_detection") + "/src/LeftPath.txt"; // 创建文件输出流对象
    outfileL.open(file_path, std::ios::out | std::ios::app);
    if (!outfileL)
    {
      std::cerr << "Failed to open the file: " << file_path << "\n";
      return;
    }
    for (const auto &pose : ros_path_.poses)
    {
      outfileL << "LeftPath: "
               << pose.pose.position.x << " "
               << pose.pose.position.y << "\n";
    }
    outfileL.close();
  }

  void Skidpad_detection::SaveLastPath()
  {
    std::ofstream outfileLast;
    const std::string file_path = ros::package::getPath("skidpad_detection") + "/src/LastPath.txt"; // 创建文件输出流对象
    outfileLast.open(file_path, std::ios::out | std::ios::app);
    if (!outfileLast)
    {
      std::cerr << "Failed to open the file: " << file_path << "\n";
      return;
    }
    for (const auto &pose : ros_path_.poses)
    {
      outfileLast << "LastPath: "
                  << pose.pose.position.x << " "
                  << pose.pose.position.y << "\n";
    }
    outfileLast.close();
  }
  // hope success
} // end namespacd fsaca
