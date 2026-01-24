#ifndef CONTROL_MY_H
#define CONTROL_MY_H

#include <ros/ros.h>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <iterator>
#include <limits>
#include <string.h>
#include <std_msgs/Bool.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float64MultiArray.h>

#include "autodrive_msgs/HUAT_Asensing.h"
#include "autodrive_msgs/HUAT_CarState.h"
#include "autodrive_msgs/HUAT_ControlCommand.h"
#include "autodrive_msgs/HUAT_PathLimits.h"
#include "autodrive_msgs/HUAT_VehicleCmd.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

#define pi 3.1415926535

namespace control
{
    struct position
    {
        double x;
        double y;
    };

    class Car
    {
    public:
        ros::NodeHandle nh;
        ros::Subscriber sub_path; // 订阅规划路径
        ros::Subscriber sub_pose; // 订阅车辆位姿
        ros::Subscriber sub_last; // 订阅中止判断

        ros::Publisher pub_cmd; // 发布控制命令

        std::vector<position> path_coordinate; // 路径坐标

        autodrive_msgs::HUAT_VehicleCmd stop_cmd; // 车辆中止指令

        int car_mode = 0; // 车辆模式

        double car_x, car_y;     // 汽车位置
        double car_veloc = 0.0;  // 汽车速度
        double car_fangle = 0.0; // 汽车偏航角
        double j, k;
        int tar;
        int now;
        int num;

        double car_length;         // 汽车长度
        double car_front_axle;     // 汽车前轴长
        double car_rear_axle;      // 汽车后轴长
        double steering_delta_max; // 最大转向角
        double steering_delta_min; // 最小转向误差

        double lookhead;                     // 车辆前视距离
        double angle_kv, angle_kl;           // 前视距离相关系数
        double angle_kp, angle_ki, angle_kd; // 角度PID控制相关系数
        double veloc_kp, veloc_ki, veloc_kd; // 速度PID控制相关系数
        double angle_integra = 0.0;          // 角度积分
        double veloc_integra = 0.0;          // 速度积分

        bool get_last_judge = false; // 检测是否订阅到中止指令
        bool get_pose_judge = false; // 检测是否订阅到车辆位姿
        bool get_path_judge = false; // 检测是否订阅到规划路径
        Eigen::Affine3d local_tf;    // 将路径转换到车辆坐标系的转换矩阵

        void get_param(std::string);

        double distance_square(double x1, double y1, double x2, double y2); // 计算两点距离的平方

        double angle_range(double alpha);   // 将角度转换到pi到-pi之间
        double angle_pid(double delta);     // 角度的pid控制

        virtual int get_steering() = 0;     // 获得方向盘转角
        virtual int get_braking() = 0;      // 获得刹车比例
        virtual int get_pedalling() = 0;    // 获得油门比例
        virtual int get_status() = 0;       // 获得比赛状态

        void file_write(int i);             // 写入车辆状态

        void control_cmd(autodrive_msgs::HUAT_VehicleCmd &cmd); // 发布车辆控制
    };
}

#endif