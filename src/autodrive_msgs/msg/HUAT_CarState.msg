std_msgs/Header header

geometry_msgs/Pose2D car_state # Position in x, y, theta
geometry_msgs/Point car_state_front
geometry_msgs/Point car_state_rear

float32 V   # 纵向速度 [m/s]
float32 W   # 偏航角速度 [rad/s] (deprecated, use Wz)
float32 A   # 加速度 [m/s^2] (deprecated, use Ax)

# FSSIM风格扩展状态
float32 Vy  # 横向速度 [m/s]
float32 Wz  # 偏航角速度 [rad/s]
float32 Ax  # 纵向加速度 [m/s^2]
float32 Ay  # 横向加速度 [m/s^2]
