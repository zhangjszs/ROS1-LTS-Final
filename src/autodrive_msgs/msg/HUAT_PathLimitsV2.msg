std_msgs/Header header            # header.stamp = 输入数据时间戳 (锥桶/车辆状态)
time stamp                        # stamp = 发布墙钟时间 (msg 生成时间)

# 路径元信息
uint8 mode                        # 规划模式
uint8 MODE_UNKNOWN = 0
uint8 MODE_ACCEL = 1              # 加速赛
uint8 MODE_SKIDPAD = 2            # 绕桩赛
uint8 MODE_TRACK = 3              # 高速循迹
uint8 MODE_AUTOCROSS = 4          # 避障赛

bool replan                       # 路径变化标志: true=路径与上帧不同, false=路径未变
uint8 quality_level               # 路径质量等级: 0=最低, 3=最高

# 轨迹点数组 (所有数组长度一致)
geometry_msgs/Point[] path        # 全局坐标下的路径点 (x, y, z=0)
float64[] s                       # 每个点的累积弧长 [m]
float64[] yaw                     # 每个点的航向角 [rad], 东向为0, 逆时针为正
float64[] curvatures              # 每个点的曲率 [1/m], 左转为正
float64[] target_speeds           # 每个点的目标速度 [m/s]
float64[] target_accels           # 每个点的目标加速度 [m/s²]
float64[] time_to_point           # 每个点的预计到达时间 [s], 相对于当前时刻

# 赛道边界信息
autodrive_msgs/HUAT_Cone[] left_boundary  # 左侧赛道边界锥桶
autodrive_msgs/HUAT_Cone[] right_boundary # 右侧赛道边界锥桶

# 附加信息
float64 lookahead_distance        # 当前建议前视距离 [m]
float64 max_available_speed       # 当前全局最大可用速度 [m/s]
float64 track_width               # 赛道平均宽度 [m]
bool finish_line_detected         # 是否检测到终点线
