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
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    RPY rpy;

};

#endif
