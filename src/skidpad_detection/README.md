# skidpad_detection

The skidpad_detection class is designed to detect obstacles on a simulated skidpad track and generate an optimal path to navigate through it.

It subscribes to two ROS topics:

- /cone_position - receives point cloud data of detected cones representing obstacles.

- /Carstate - receives current state of the car including position and orientation.

In the constructor, parameters are loaded from ROS parameter server. Callbacks are registered to the subscribed topics. Some variables are initialized. 

The main logic happens in skidpadCallback() and runAlgorithm():

- skidpadCallback() processes the incoming point cloud. It filters out unnecessary points, finds the 4 outer cones and calculates the angle between them. This angle is used later to transform paths.

- runAlgorithm() contains the main logic to determine and publish driving paths segment by segment. It checks flags to know the current state, and publishes transformed paths according to the current stage. Paths consist of straight lines and arcs around cones.

Some key steps:

- Point cloud is filtered to remove unnecessary points.

- Outer cones are extracted and used to calculate orientation angle. 

- Paths are planned and transformed using the calculated angle.

- Path segments are published based on logic flags indicating current stage.

- Position info is tracked to determine when to switch stages.

The code implements an effective algorithm to dynamically generate feasible paths in a simulated skidpad environment by subscribing to sensor inputs and reacting accordingly.

主要步骤如下:

1. 初始化节点,订阅话题获取锥桶位置(/cone_position)和车辆状态(/Carstate)。
2. 定义参数,包括车到激光雷达距离、目标点坐标、距离阈值等。
3. 接收到锥桶消息后,进行点云预处理(限制x、y方向范围)。
4. 从预处理后的点云中找到4个点,确定第一个和第二个扇形的中点。
5. 计算两个中点连线的角度,记录为at2_angle_mid。
6. 当匹配到4个点后,设置标志位at2_angle_calced为true。
7. 在运行算法函数runAlgorithm中:
    1. 如果匹配到4个点,发布第一段直线行驶路径。
    2. 如果检测到进入环岛(changFlag为true),根据当前阶段modeFlag,发布对应阶段的环岛路径。
    3. 发布路径后更新modeFlag,准备发布下一段路径。
8. 发布路径前,进行坐标变换,将路径旋转至合适角度。
9. 保存各段路径到文件。
10. 其他功能还包括:
    - 参数加载函数loadParameters。
    - 位置回调保存车辆状态函数positionback。
    - 检测是否进入/离开圆环的状态函数。
	1. 坐标变换的目的:是为了将检测到的路径点坐标从传感器坐标系转换到车辆坐标系,方便控制器直接使用。
	2. positionback函数的作用:是保存车辆状态作为日志,本算法中并没有用到这些数据,可能是为了后续功能扩展预留的。
	3. 路径发布的频率和时机:main函数中通过ros::Rate对象控制循环频率,在每次循环中调用。skidpad检测算法的runAlgorithm函数,从而定期触发路径发布。
	4. 进入圆环检测逻辑:
    - changFlag表示是否已接近圆环目标位置(进入圆环)
    - haschanged表示是否已离开圆环目标位置(完成圆环岛)
    - 进入圆环时设置changFlag=true。
    - 离开圆环时先设置haschanged=true,然后检测到haschanged=true时才发布离开圆环的路径
    - 这个检测逻辑是未来确保每次只发布一次路径。
