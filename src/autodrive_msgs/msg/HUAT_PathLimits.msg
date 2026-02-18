std_msgs/Header header            # header.stamp = 输入数据时间戳 (锥桶/车辆状态)
time stamp                        # stamp = 发布墙钟时间 (msg 生成时间)
geometry_msgs/Point[] path        # 路径点上的数组
autodrive_msgs/HUAT_TrackLimits tracklimits # 赛道限制信息
bool replan                      # B14: 路径变化标志
                                 # high_speed模式: true=路径与上帧不同, false=路径未变
                                 # line/skidpad模式: 固定true (兼容历史行为)
float64[] target_speeds          # per-point target speed [m/s], 1:1 with path[]
float64[] curvatures             # per-point curvature [1/m], 1:1 with path[]

