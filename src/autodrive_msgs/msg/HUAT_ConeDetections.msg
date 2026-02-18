Header header
geometry_msgs/Point32[] points # coordinate of cone in [x, y]
geometry_msgs/Point32[] maxPoints # coordinate of cone in [x, y]
geometry_msgs/Point32[] minPoints # coordinate of cone in [x, y]
sensor_msgs/PointCloud2[] pc
sensor_msgs/PointCloud2 pc_whole

float32[] confidence
float32[] obj_dist
int32[] track_ids # per-cone tracker ID (-1 = untracked)
std_msgs/String color # B15: [DEPRECATED] 旧版全局颜色字段, 仅visualization兼容回退使用
                       # 新代码请使用 color_types[] (per-cone枚举)
uint8[] color_types # B15: per-cone颜色枚举 (统一标准):
                    # 0=BLUE, 1=YELLOW, 2=ORANGE_SMALL, 3=ORANGE_BIG, 4=NONE, 5=RED
                    # LiDAR-only管线仅输出 2/3/4; 0/1/5 保留给相机融合
