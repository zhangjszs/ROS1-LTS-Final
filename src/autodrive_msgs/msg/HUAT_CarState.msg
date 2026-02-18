std_msgs/Header header

geometry_msgs/Pose2D car_state # Position in x, y, theta
geometry_msgs/Point car_state_front
geometry_msgs/Point car_state_rear

float32 V   # 纵向速度 [m/s]
float32 W   # B16: [DEPRECATED target-removal:2026-12] 偏航角速度标量 → 替代: Wz
float32 A   # B16: [DEPRECATED target-removal:2026-12] 加速度标量 → 替代: sqrt(Ax²+Ay²)

# FSSIM风格扩展状态 (B16: 新代码应使用以下字段)
float32 Vy  # 横向速度 [m/s]
float32 Wz  # 偏航角速度 [rad/s] (替代 W)
float32 Ax  # 纵向加速度 [m/s^2] (替代 A)
float32 Ay  # 横向加速度 [m/s^2]
