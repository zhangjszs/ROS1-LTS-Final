Header header
geometry_msgs/Point32[] points # coordinate of cone in [x, y]
geometry_msgs/Point32[] maxPoints # coordinate of cone in [x, y]
geometry_msgs/Point32[] minPoints # coordinate of cone in [x, y]
sensor_msgs/PointCloud2[] pc
sensor_msgs/PointCloud2 pc_whole

float32[] confidence
float32[] obj_dist
std_msgs/String color # color of cone, 'b' = blue, 'y' = yellow, 'r' = red 'n'=None
uint8[] color_types # per-cone color: 0=BLUE, 1=YELLOW, 2=ORANGE_SMALL, 3=ORANGE_BIG, 4=NONE
