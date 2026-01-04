#include <ros/ros.h>
#include <iostream>
#include <mutex>
#include <deque>
#include <thread>

#ifndef MULTI_SENSOR_FUSION_IMU_DATA_HPP_
#define MULTI_SENSOR_FUSION_IMU_DATA_HPP_

class IMUData {
  public:
    struct LinearAcceleration {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };
    struct RPY{
      double heading = 0.0;
      double pitch = 0.0;
      // double roll = 0.0;
    };

    // class Orientation {
    //   public:
    //     double x = 0.0;
    //     double y = 0.0;
    //     double z = 0.0;
    //     double w = 0.0;
      
    //   public:
    //     void Normlize() {
    //       double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
    //       x /= norm;
    //       y /= norm;
    //       z /= norm;
    //       w /= norm;
    //     }
    // };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    RPY rpy;
    // Orientation orientation;
    
    
  public:

    // mutex(const mutex&) = delete;
};

#endif