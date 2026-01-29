# HUAT_InsP2.msg
# INS5711DAA 高精度组合导航系统数据
# 
# 坐标系说明:
# - 车体相关量(角速度/加速度/姿态角): FRD (Front-Right-Down)
#   X轴: 车头前进方向为正
#   Y轴: 车体右侧为正
#   Z轴: 向地面/地心为正(向下)
# - 导航速度: NED (North-East-Down)
#
# 参考文档: INS5711DAA 使用手册 V1.05

Header header

# ========== GPS 时间 ==========
uint16 Week          # GPS 周
float32 Time         # GPS 周内秒 (s)

# ========== 姿态角 (度) ==========
# 注意: Heading 的参考系依赖于 INS_Status
#   - Status=1: 以上电姿态初始化时为 0°
#   - Status=2: 以正北为 0°，正东 90°，正西 -90°
float32 Heading      # 航向角/方位角 (度), 正北0°顺时针为正
float32 Pitch        # 俯仰角 (度), 抬头为正
float32 Roll         # 横滚角 (度), 右倾为正

# ========== 角速度 (rad/s) ==========
# 设备原始输出: deg/s
# 此消息中已转换为: rad/s (通过 ×π/180)
# 坐标系: FRD 车体系
float32 gyro_x       # 绕X轴角速度 (Roll rate, rad/s)
float32 gyro_y       # 绕Y轴角速度 (Pitch rate, rad/s)
float32 gyro_z       # 绕Z轴角速度 (Yaw rate, rad/s)

# ========== 加速度 (m/s²) ==========
# 设备原始输出: g (重力加速度单位)
# 此消息中已转换为: m/s² (通过 ×9.7883105)
# 坐标系: FRD 车体系
# 注: 静止时 acc_z ≈ 9.78 m/s² (向下为正)
float32 acc_x        # X轴加速度 (前向, m/s²)
float32 acc_y        # Y轴加速度 (右向, m/s²)
float32 acc_z        # Z轴加速度 (下向, m/s²)

# ========== 位置 (WGS84) ==========
# 注: 仅在 INS_Status=2 时有效
float64 Lat          # 纬度 (度)
float64 Lon          # 经度 (度)
float32 Altitude     # 高度 (m)

# ========== 速度 (m/s, NED坐标系) ==========
# NED: North-East-Down 导航坐标系 (向下为正)
float32 Ve           # 东向速度 (m/s)
float32 Vn           # 北向速度 (m/s)
float32 Vd           # 地向速度 (m/s), 向下为正

# ========== 双天线基线 ==========
float32 Base         # 基线长度 (m), 建议 ≥ 1m

# ========== 状态信息 ==========
uint8 NSV1           # 主天线卫星数 (自定义字段)
uint8 NSV2           # 副天线卫星数 (自定义字段)

# INS 总体状态
# 0 = NONE (未初始化)
# 1 = 姿态初始化完成 (Heading 以初始方向为参考)
# 2 = 组合导航正常 (位置/速度/姿态均有效, Heading 以正北为参考)
uint8 Status

uint8 Age            # 差分龄期 (0.1s 单位, 需 ×0.1 转换为秒)
uint8 War            # 警告标志 (自定义字段)
