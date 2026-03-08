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
                    # 0=BLUE, 1=YELLOW_SMALL, 2=YELLOW_BIG, 3=RED, 4=NONE
                    # 注意：
                    # - 视觉端输出统一的YELLOW (1)，不区分大小
                    # - LiDAR根据几何尺寸将YELLOW细分为YELLOW_SMALL (1)或YELLOW_BIG (2)
                    # - 融合时，视觉YELLOW + LiDAR尺寸 -> YELLOW_SMALL/YELLOW_BIG
