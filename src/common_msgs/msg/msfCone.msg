# cone information contrain color point3d and cone type
Header header
geometry_msgs/Point32[] points  # coordinate of cone in [x, y]
sensor_msgs/PointCloud2[] pc
sensor_msgs/PointCloud2 pc_whole
float32[] obj_dist
std_msgs/String color           # color of cone, 'b' = blue, 'y' = yellow, 'r' = red 'n'=None
