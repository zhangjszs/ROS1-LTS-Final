std_msgs/Header header
time stamp
geometry_msgs/Point[] path #路径点上的数组
autodrive_msgs/HUAT_TrackLimits tracklimits #赛道限制信息
bool replan #
float64[] target_speeds # per-point target speed [m/s], 1:1 with path[]
float64[] curvatures # per-point curvature [1/m], 1:1 with path[]

