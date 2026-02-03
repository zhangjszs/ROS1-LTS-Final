# Simulation state message for vehicle dynamics simulation
std_msgs/Header header

# Vehicle pose (x, y, heading)
geometry_msgs/Pose2D pose

# Velocity components (vx, vy in vehicle frame, omega is yaw rate)
geometry_msgs/Vector3 velocity

# Control inputs
float32 steering_angle    # rad
float32 throttle          # 0-1
float32 brake             # 0-1

# Simulation time
float64 sim_time          # seconds
