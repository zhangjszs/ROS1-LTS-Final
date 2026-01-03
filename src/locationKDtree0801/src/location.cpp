#include "location_head.hpp"

namespace coordinate
{
    Location::Location(ros::NodeHandle &nh) : nh_(nh)
    {
        //ROS_INFO("load parameters");
        loadParameters();
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

        INS_sub = nh_.subscribe<common_msgs::HUAT_ASENSING>("/pbox_pub/Ins",10,&Location::doINSMsg,this);
        cone_sub = nh_.subscribe<common_msgs::Cone>("/cone_position",99999999,&Location::doConeMsg,this);
        //ROS_INFO("callback!!!");
        carState_pub = nh_.advertise<common_msgs::HUAT_Carstate>("/Carstate",10);
        map_pub = nh_.advertise<common_msgs::HUAT_map>("/coneMap",10);
        globalMap_pub = nh_.advertise<sensor_msgs::PointCloud2>("/globalMapOnly",10);

        carMarker_pub = nh_.advertise<visualization_msgs::Marker>("/carBody",10);
        wholeMarker_pub = nh_.advertise<visualization_msgs::Marker>("/whole",10);
        coneMarker_pub = nh_.advertise<visualization_msgs::Marker>("/coneMarker",10);
    
    }

    //加载参数:从参数服务器上读取相关参数,前后轮轴中心及雷达到IMU的距离等,读取失败使用默认值。
    void Location::loadParameters()
    {
        ROS_INFO("location loading Parameters");
        
        if (!nh_.param("length/lidarToIMUDist", lidarToIMUDist_, 1.87))
        {
            //IMU质心到激光雷达的距离
            ROS_WARN_STREAM("Did not load lidarToIMUDist. Standard value is: " << lidarToIMUDist_);
        }

        if (!nh_.param("length/frontToIMUdistanceX", frontToIMUdistanceX_, 0.0))
        {
            //前轮轴中心到IMU质心的x轴距离
            ROS_WARN_STREAM("Did not load frontToIMUdistanceX. Standard value is: " << frontToIMUdistanceX_);
        }

        if (!nh_.param("length/frontToIMUdistanceY", frontToIMUdistanceY_, 0.0))
        {
            //前轮轴中心到IMU质心的y轴距离
            ROS_WARN_STREAM("Did not load frontToIMUdistanceY. Standard value is " << frontToIMUdistanceY_);
        }

        if (!nh_.param("length/frontToIMUdistanceZ", frontToIMUdistanceZ_, 0.0))
        {
            //前轮轴中心到IMU质心的z轴距离
            ROS_WARN_STREAM("Did not load frontToIMUdistanceZ. Standard value is: " << frontToIMUdistanceZ_);
        }

        if (!nh_.param("length/rearToIMUdistanceX", rearToIMUdistanceX_, 0.0))
        {
            //后轮轴中心到IMU质心的x轴距离
            ROS_WARN_STREAM("Did not load rearToIMUdistanceX. Standard value is " << rearToIMUdistanceX_);
        }

        if (!nh_.param("length/rearToIMUdistanceY", rearToIMUdistanceY_, 0.0))
        {
            //后轮轴中心到IMU质心的y轴距离
            ROS_WARN_STREAM("Did not load rearToIMUdistanceY. Standard value is " << rearToIMUdistanceY_);
        }

        if (!nh_.param("length/rearToIMUdistanceZ", rearToIMUdistanceZ_, 0.0))
        {
            //后轮轴中心到IMU质心的z轴距离
            ROS_WARN_STREAM("Did not load rearToIMUdistanceZ. Standard value is " << rearToIMUdistanceZ_);
        }

        if (!nh_.param<std::string>("location", subTopic_, "/location"))
        {
            //订阅的话题名称
            ROS_WARN_STREAM("Did not load topic name. Standard value is: " << subTopic_);
        }
        return;
    }
    //车辆状态信息处理
    void Location::doINSMsg(const common_msgs::HUAT_ASENSING::ConstPtr& msgs)
    {
        Mimu=*msgs;
        const common_msgs::HUAT_ASENSING& imu_data=Mimu;
        //ROS_WARN("callback!!!doINSMsg");    
        //saveGPS(msgs->latitude,msgs->longitude,msgs->altitude);
        //将ins消息中的赋值给Mins结构体
        Mins.east_velocity = imu_data.east_velocity;
    	Mins.north_velocity = imu_data.north_velocity;
    	Mins.ground_velocity = imu_data.ground_velocity;
        Mins.azimuth = imu_data.azimuth;//方位角

        Mins.x_angular_velocity = imu_data.x_angular_velocity;
        Mins.y_angular_velocity = imu_data.y_angular_velocity;
        Mins.z_angular_velocity = imu_data.z_angular_velocity;

        Mins.x_acc = imu_data.x_acc * 9.79;
        Mins.y_acc = imu_data.y_acc * 9.79;
        Mins.z_acc = ( imu_data.z_acc + cos(imu_data.roll) * cos(imu_data.pitch) ) * 9.79; 

        if (!isfirstINSreceived) 
        {
            //第一次接收到消息
            Carstate.car_state.theta = 0;
            oldAzimuth = imu_data.azimuth;//得到第一次的角度
            Carstate.V = sqrt(pow(Mins.east_velocity,2)+pow(Mins.north_velocity,2)+pow(Mins.ground_velocity,2));//计算线速度
            Carstate.W = sqrt(pow(Mins.x_angular_velocity,2)+pow(Mins.y_angular_velocity,2)+pow(Mins.z_angular_velocity,2));//计算角速度
            Carstate.A = sqrt(pow(Mins.x_acc,2)+pow(Mins.y_acc,2)+pow(Mins.z_acc,2));//计算线加速度 

            first_lat = imu_data.latitude;
            first_lon = imu_data.longitude;
            first_alt = imu_data.altitude;

            Carstate.car_state.x = 0;
            Carstate.car_state.y = 0;
            //saveGPS(first_lat,first_lon,first_alt);
            carState_pub.publish(Carstate);
            isfirstINSreceived = true;//表示已经收到了第一条消息。
        } 
        else 
        {
            double diff = - (imu_data.azimuth - oldAzimuth);//计算方位角的变化量
            Carstate.car_state.theta =  diff *PII/180;
            //处理角度的特殊情况
            if (Carstate.car_state.theta > PII) 
            {
                Carstate.car_state.theta -= 2 * PII;
                diff -=360;
            } 
            else if (Carstate.car_state.theta < -PII) 
            {
                Carstate.car_state.theta += 2 * PII;
                diff += 360;
            }
            //saveAngle(oldAzimuth,msgs->azimuth,diff);
            Carstate.V = sqrt(pow(Mins.east_velocity,2)+pow(Mins.north_velocity,2)+pow(Mins.ground_velocity,2));//计算车辆线速度
            Carstate.W = sqrt(pow(Mins.x_angular_velocity,2)+pow(Mins.y_angular_velocity,2)+pow(Mins.z_angular_velocity,2));//计算角速度
            Carstate.A = sqrt(pow(Mins.x_acc,2)+pow(Mins.y_acc,2)+pow(Mins.z_acc,2));//计算线加速度 
            //将当前纬度,经度,高度转换为ENU坐标下的坐标,并存储在enu_xyz数组中
            GeoDetic_TO_ENU((imu_data.latitude) * PII / 180, (imu_data.longitude) * PII / 180, imu_data.altitude,
                                first_lat * PII / 180, first_lon * PII / 180, first_alt, &enu_xyz[0]);
        }
        calcVehicleDirection(imu_data.roll, imu_data.pitch, Carstate.car_state.theta,  dir_x, dir_y, dir_z);//计算车辆的方向向量
        //可视化车辆和整个场景
        visCar();
        visWhole();
    }

    //坐标转换
    void Location::GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3])
    {
        /*
            根据经纬度及高度先转换为ECEF坐标(地心地固坐标),然后转换为ENU坐标(东北天坐标)
            创建一个 tf2::Transform 对象 transform对象,用于表示站点相对于原点的位置和方向信息
        */
        // 定义一些常量，表示参考椭球体的参数
        ROS_DEBUG("transform!!!");  
        double a, b, f, e_sq;
	    a = 6378137; // 长半轴，单位为米
	    b = 6356752.3142; // 短半轴，单位为米
	    f = (a - b) / a; // 扁率
	    e_sq = f * (2 - f); // 第一偏心率平方

	    // 计算站点（非原点）的ECEF坐标（地心地固坐标系）
	    double lat_, lon_, N, sin_lat, cos_lat, sin_lon, cos_lon, x, y, z;
	    lat_ = lat;  // 纬度，单位为度
	    lon_ = lon; // 经度，单位为度

	    sin_lat = sin(lat_); // 纬度的正弦值
	    cos_lat = cos(lat_); // 纬度的余弦值
	    sin_lon = sin(lon_); // 经度的正弦值
	    cos_lon = cos(lon_); // 经度的余弦值

        N = a / sqrt(1 - e_sq * sin_lat * sin_lat); // 卯酉圈曲率半径

	    x = (h + N) * cos_lat * cos_lon; // x坐标，单位为米
	    y = (h + N) * cos_lat * sin_lon; // y坐标，单位为米
	    z = (h + (1 - e_sq) * N) * sin_lat; // z坐标，单位为米

	    // 计算原点的ECEF坐标（地心地固坐标系）
	    double lat0_, lon0_, N0, sin_lat0, cos_lat0, sin_lon0, cos_lon0, x0, y0, z0;
	    lat0_ = lat0; // 原点的纬度，单位为度
	    lon0_ = lon0; // 原点的经度，单位为度

	    sin_lat0 = sin(lat0_); // 原点纬度的正弦值
	    cos_lat0 = cos(lat0_); // 原点纬度的余弦值
	    sin_lon0 = sin(lon0_); // 原点经度的正弦值
	    cos_lon0 = cos(lon0_); // 原点经度的余弦值
        N0 = a / sqrt(1 - e_sq * sin_lat0 * sin_lat0); // 原点卯酉圈曲率半径

	    x0 = (h0 + N0) * cos_lat0 * cos_lon0; // 原点x坐标，单位为米
	    y0 = (h0 + N0) * cos_lat0 * sin_lon0; // 原点y坐标，单位为米
	    z0 = (h0 + (1 - e_sq) * N0) * sin_lat0; // 原点z坐标，单位为米

	    // 计算站点相对于原点的ECEF坐标差
	    double xd, yd, zd;
	    xd = x - x0;
	    yd = y - y0;
	    zd = z - z0;

	    // 计算站点的ENU坐标（本地东北天坐标系）

	    enu_xyz[0] = -sin_lon0 * xd + cos_lon0 * yd; // 东方向坐标，单位为米
	    enu_xyz[1] = (-cos_lon0 * xd - sin_lon0 * yd) * sin_lat0 + cos_lat0 * zd; // 北方向坐标，单位为米
	    enu_xyz[2] = cos_lat0 * cos_lon0 * xd + cos_lat0 * sin_lon0 * yd + sin_lat0 * zd; // 天空方向坐标,平面

        front_wheel[0] = enu_xyz[0] + frontToIMUdistanceX_;
        front_wheel[1] = enu_xyz[1] + frontToIMUdistanceY_;
        front_wheel[1] = enu_xyz[2] + frontToIMUdistanceZ_;

        rear_wheel[0] = enu_xyz[0] + rearToIMUdistanceX_;
        rear_wheel[1] = enu_xyz[1] + rearToIMUdistanceY_;
        rear_wheel[1] = enu_xyz[2] + rearToIMUdistanceZ_;

        tf2::Transform transform;//创建一个 tf2::Transform 对象 transform
        transform.setOrigin(tf2::Vector3( 0,0,0));//设置变换的原点为 (0, 0, 0)
        tf2::Quaternion q;//创建四元数对象 q
        //向该对象设置欧拉角,这个对象可以将欧拉角转换成四元数
        q.setRPY(0, 0,  (oldAzimuth - 90) * (PII / 180));//设置了方位角,单位为弧度
        transform.setRotation(q);//将这个四元数设置为 transform 的旋转部分

        tf2::Vector3 state(enu_xyz[0],enu_xyz[1],0);//使用 enu_xyz 数组的前两个值创建一个 tf2::Vector3 对象 state，并将 z 坐标设置为 0
        tf2::Vector3 front_state(front_wheel[0],front_wheel[1],front_wheel[2]);
        tf2::Vector3 rear_state(rear_wheel[0],rear_wheel[1],rear_wheel[2]);

        state = transform * state;//将变换 transform 应用到向量 state 上。结果是一个新的向量，表示 state 在应用了旋转变换后的坐标
        front_state = transform * front_state;
        rear_state = transform * rear_state;

        // 更新车辆及前轮后轮状态的x和y坐标
        Carstate.car_state.x = state.x();
        Carstate.car_state.y = state.y();

        Carstate.car_state_front.x = front_state.x();
        Carstate.car_state_front.y = front_state.y();
        Carstate.car_state_front.z = front_state.z();

        Carstate.car_state_rear.x = rear_state.x();
        Carstate.car_state_rear.y = rear_state.y();
        Carstate.car_state_rear.z = rear_state.z();

        saveCarstate(Carstate.car_state.x,Carstate.car_state.y);
        //saveState(Carstate.car_state.x,Carstate.car_state.y,Carstate.car_state.theta,
        //            Carstate.car_state_front.x,Carstate.car_state_front.y,Carstate.car_state_front.z,
        //            Carstate.car_state_rear.x,Carstate.car_state_rear.y,Carstate.car_state_rear.z);

        // 发布车辆状态的消息，并设置接收标志位为真
        carState_pub.publish(Carstate);
        isfirstINSreceived = true;//表示已经收到了第一条消息。
    }

    // 计算车辆前进方向单位向量的函数
    void Location::calcVehicleDirection(double roll, double pitch, double yaw,  double &x, double &y, double &z) 
    {
        ROS_DEBUG("calc_vehicle_direction");  
        // 将角度值转换为弧度
        tfRoll = DEG2RAD(roll);
        tfPitch = DEG2RAD(pitch);
        x = cos(tfPitch) * cos(yaw);//dir_x
        y = sin(tfRoll) * sin(tfPitch) * cos(yaw) + cos(tfRoll) * sin(yaw);//dir_y
        z = -tfRoll * sin(yaw)  + tfPitch * sin(yaw);//dir_z
    }

    //锥筒信息处理
    void Location::doConeMsg(const common_msgs::Cone::ConstPtr& msgs)
    {
        //ROS_DEBUG("callback!!!doConMsg");    
        if(!isfirstINSreceived)
        {
            ROS_WARN("INS 数据没更新!");
            return;
        }
        //检查是否收到锥筒信息
        if (msgs->points.empty())
        {
            ROS_WARN("锥桶坐标为空!");
            return;
        }
        Ymap.cone.clear();
        
        // 准备进行坐标系转换,将锥筒从lider坐标系转换到全局坐标系
        tf2::Transform transform;

        transform.setOrigin(tf2::Vector3(0, 0, 0));
        tf2::Quaternion q;
        q.setRPY(0, 0, (Carstate.car_state.theta));

        transform.setRotation(q);

        tf2::Vector3 lidarPosition(lidarToIMUDist_, 0, 0);
        lidarPosition = transform * lidarPosition;
        tf2::Vector3 pos(Carstate.car_state.x, Carstate.car_state.y, 0);
        //saveCarstate(Carstate.car_state.x,Carstate.car_state.y);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (!firstConeMsg)
        {
            //如果是第一次接收到锥筒信息
            std::cout << "接受第一帧信息" << std::endl;
            firstConeMsg = true;
            for (int i = 0; i < msgs->points.size(); i++)
            {
                tf2::Vector3 pointToTf(msgs->points[i].x, msgs->points[i].y, 0);
                tf2::Vector3 pointToTf_b = transform * pointToTf;

                Ycone.position_baseLink.x = pointToTf.x() + lidarToIMUDist_;
                Ycone.position_baseLink.y = pointToTf.y();
                Ycone.position_baseLink.z = msgs->points[i].z;

                Ycone.position_global.x = pointToTf_b.x() + pos.x() + lidarPosition.x();
                Ycone.position_global.y = pointToTf_b.y() + pos.y() + lidarPosition.y();
                Ycone.position_global.z = msgs->points[i].z;

                // 第一个到的消息不需要检测，直接给新id
                Ycone.id = getNewId();
                Ymap.cone.push_back(Ycone);
                pcl::PointXYZ point;
                point.x = Ycone.position_global.x;
                point.y = Ycone.position_global.y;
                point.z = Ycone.position_global.z;

                cloud->push_back(point);
                cloud->width = cloud->points.size();
                cloud->height = 1;
                point_ids.push_back(id);
                global_cloud->push_back(point);//锥筒信息添加到地图和点云中
                //savePoint(Ycone.position_global.x, Ycone.position_global.y, Ycone.position_global.z, Ycone.id); // 给-号是因为坐标系y轴副方向为正
                visCone(Ycone.position_global.x, Ycone.position_global.y, Ycone.position_global.z, Ycone.id);
            }
        }
        else
        {
            //如果不是第一次接收到锥筒信息,需要检查是否在地图中已经存在一个很接近的点,如存在则更新,若不存在,则创建一个新的点并添加到地图中
            for (int i = 0; i < msgs->points.size(); i++)
            {
                //遍历当前帧的所有锥筒点
                tf2::Vector3 pointToTf(msgs->points[i].x, msgs->points[i].y, 0);
                tf2::Vector3 pointToTf_b = transform * pointToTf;

                Ycone.position_baseLink.x = pointToTf.x() + lidarToIMUDist_;
                Ycone.position_baseLink.y = pointToTf.y();
                Ycone.position_baseLink.z = msgs->points[i].z;

                Ycone.position_global.x = pointToTf_b.x() + pos.x() + lidarPosition.x();
                Ycone.position_global.y = pointToTf_b.y() + pos.y() + lidarPosition.y();
                Ycone.position_global.z = msgs->points[i].z;

                pcl::PointXYZ point;
                point.x = Ycone.position_global.x;
                point.y = Ycone.position_global.y;
                std::vector<int> pointIdxNKNSearch(1);//存储邻点的索引结果的向量
                std::vector<float> pointNKNSquaredDistance(1);//存储邻点与查询点之间的平方距离结果的向量
/*
对于每个点,使用 KD 树搜索最近的已记录点
如果找到的最近点在一定距离阈值内,则更新该点的位置信息
如果没有找到足够近的点,则认为是一个新的锥桶,添加到地图和点云中
*/
                if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                {
                    //ROS_WARN("kdtree!!!");
                /**********************修改这里V改变认为是同一锥筒的阈值*****************************/
                    if (pointNKNSquaredDistance[0] <= 2.5 && !(pointIdxNKNSearch[0] >= cloud->size()))
                    {   // 修改距离，在这个距离内认为是同一点
                        // 找到点就修正其值
                    /**********************修改这里^改变认为是同一锥筒的阈值*****************************/

                        cloud->points[pointIdxNKNSearch[0]].x = (cloud->points[pointIdxNKNSearch[0]].x + point.x) / 2.0;
                        cloud->points[pointIdxNKNSearch[0]].y = (cloud->points[pointIdxNKNSearch[0]].y + point.y) / 2.0;
                        Ycone.position_global.x = cloud->points[pointIdxNKNSearch[0]].x;
                        Ycone.position_global.y = cloud->points[pointIdxNKNSearch[0]].y;
                        Ycone.id = point_ids[pointIdxNKNSearch[0]];
                        Ymap.cone.push_back(Ycone);

                        point.x = Ycone.position_global.x;
                        point.y = Ycone.position_global.y;
                        global_cloud->push_back(point);
                        //ROS_WARN("chance!!!");
                        visCone(Ycone.position_global.x, Ycone.position_global.y, Ycone.position_global.z, Ycone.id);
                        //savePoint(Ycone.position_global.x,Ycone.position_global.y,Ycone.position_global.z,Ycone.id);
                    }
                    else
                    {
                        // 找到了最近邻点,但距离超过阈值，生成新的 ID
                        int id = getNewId();
                        Ycone.id = id;
                        Ymap.cone.push_back(Ycone);
                        cloud->push_back(point);
                        cloud->width = cloud->points.size();
                        cloud->height = 1;
                        point_ids.push_back(id);
                        global_cloud->push_back(point);
                        //ROS_WARN("threshold generate!!!");
                        visCone(Ycone.position_global.x, Ycone.position_global.y, Ycone.position_global.z, Ycone.id);
                        //savePoint(Ycone.position_global.x, Ycone.position_global.y, Ycone.position_global.z, Ycone.id); // 给-号是因为坐标系y轴副方向为正
                    }
                }
                else
                {
                    // 没有找到距离在 0.05以内的最近邻点，生成新的 ID
                    int id = getNewId();
                    Ycone.id = id;
                    Ymap.cone.push_back(Ycone);
                    cloud->push_back(point);
                    cloud->width = cloud->points.size();
                    cloud->height = 1;
                    point_ids.push_back(id);
                    global_cloud->push_back(point);
                    //ROS_WARN("generate!!!");
                    visCone(Ycone.position_global.x, Ycone.position_global.y, Ycone.position_global.z, Ycone.id);
                    //savePoint(Ycone.position_global.x, Ycone.position_global.y, Ycone.position_global.z, Ycone.id); // 给-号是因为坐标系y轴副方向为正
                }
            }
        }
        //saveCone(Ycone);
        kdtree.setInputCloud(cloud);//更新KD树以反映地图中锥筒的变化
        // std::sort(Ymap.cone.begin(), Ymap.cone.end(),
        //           [](const common_msgs::HUAT_cone& a, const common_msgs::HUAT_cone& b){
        //               return a.id < b.id;
        //           });
        map_pub.publish(Ymap);//发布更新后的地图信息

        //saveMap(Ymap);

        sensor_msgs::PointCloud2 global_cloud_msg;
        pcl::toROSMsg(*global_cloud, global_cloud_msg);
        global_cloud_msg.header.frame_id = "velodyne";
        global_cloud_msg.header.stamp = ros::Time::now();

        // 发布PointCloud2消息
        globalMap_pub.publish(global_cloud_msg);
    }

    //用于生成新的锥筒ID
    int Location::getNewId()
    {
        id++;
        return id;
    }

    /*
    p1         p2点的顺序
    p3         p4
    */
    //车身可视化
    void Location::visCar()
    {
        // 创建一个线标记
        visualization_msgs::Marker marker;
        marker.header.frame_id = "velodyne";//标记位于 Velodyne 雷达坐标系下。
        marker.header.stamp = ros::Time();
        marker.ns = "rectangle"; //这个标记的命名空间为rectangle
        marker.id = 0; //这是第一个标记
        marker.type = visualization_msgs::Marker::LINE_STRIP; //这是一个由一系列线段组成的标记。
        marker.action = visualization_msgs::Marker::ADD; //添加一个新标记

        // 将车的位置转换为标记坐标系中的位置,用于表示车辆相对于Velodyne雷达的位置和方向
        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3( lidarToIMUDist_ * cos((Carstate.car_state.theta)),  lidarToIMUDist_ * sin((Carstate.car_state.theta)), 0)); // 偏移量
        tf2::Quaternion q;
        q.setRPY(0, 0, (Carstate.car_state.theta));
        transform.setRotation(q);
        tf2::Vector3 pos(Carstate.car_state.x, Carstate.car_state.y, 0);

        // 计算每个点相对于车的位置的向量
        tf2::Vector3 v1(0, 0.25, 0);
        tf2::Vector3 v2(0, -0.25, 0);
        tf2::Vector3 v3(-1.5, 0.25, 0);
        tf2::Vector3 v4(-1.5, -0.25, 0);

        // 将每个向量旋转相同的角度
        v1 = transform * v1;
        v2 = transform * v2;
        v3 = transform * v3;
        v4 = transform * v4;

        // 将每个向量添加到标记中
        geometry_msgs::Point p1, p2, p3, p4;
        p1.x = pos.x() + v1.x();
        p1.y = pos.y() + v1.y();
        p1.z = 0;
        p2.x = pos.x() + v2.x();
        p2.y = pos.y() + v2.y();
        p2.z = 0;
        p3.x = pos.x() + v3.x();
        p3.y = pos.y() + v3.y();
        p3.z = 0;
        p4.x = pos.x() + v4.x();
        p4.y = pos.y() + v4.y();
        p4.z = 0;

        // 添加四个点到线标记中
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p4);

        // 设置标记的尺寸（线的宽度）
        marker.scale.x = 0.3;

        // 设置标记的颜色和透明度
        marker.color.a = 1; // 透明度为50%
        marker.color.r = 1.0; // 红色
        marker.color.g = 0.0; // 绿色
        marker.color.b = 0.0; // 蓝色

        // 发布标记
        carMarker_pub.publish(marker);
    }

    //车轮可视化
    void Location::visWhole()
    {
        visualization_msgs::Marker wheel_lf, wheel_rf, wheel_lr, wheel_rr;//创建对象用于表示四个轮子
        double angle ;
        double move;

        //对每个轮子标记设置基本属性
        wheel_lf.header.frame_id = "velodyne";
        wheel_lf.header.stamp = ros::Time::now();
        wheel_lf.ns = "car_marker";
        wheel_lf.id = 1;
        wheel_lf.type = visualization_msgs::Marker::CYLINDER;
        wheel_lf.action = visualization_msgs::Marker::ADD;

        wheel_rf.header.frame_id = "velodyne";
        wheel_rf.header.stamp = ros::Time::now();
        wheel_rf.ns = "car_marker";
        wheel_rf.id = 2;
        wheel_rf.type = visualization_msgs::Marker::CYLINDER;
        wheel_rf.action = visualization_msgs::Marker::ADD;

        wheel_lr.header.frame_id = "velodyne";
        wheel_lr.header.stamp = ros::Time::now();
        wheel_lr.ns = "car_marker";
        wheel_lr.id = 3;
        wheel_lr.type = visualization_msgs::Marker::CYLINDER;
        wheel_lr.action = visualization_msgs::Marker::ADD;

        wheel_rr.header.frame_id = "velodyne";
        wheel_rr.header.stamp = ros::Time::now();
        wheel_rr.ns = "car_marker";
        wheel_rr.id = 4;
        wheel_rr.type = visualization_msgs::Marker::CYLINDER;
        wheel_rr.action = visualization_msgs::Marker::ADD;

        //用于表示相对于Velodyne雷达的位置和方向
        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3( lidarToIMUDist_ * cos(Carstate.car_state.theta),  lidarToIMUDist_ * sin(Carstate.car_state.theta), 0)); // 偏移量
        tf2::Quaternion qq;
        qq.setRPY(0, 0, (Carstate.car_state.theta));
        transform.setRotation(qq);

        // 计算每个轮子相对于车的位置的向量
        tf2::Vector3 v1(-0.2, 0.35, 0);
        tf2::Vector3 v2(-0.2, -0.35, 0);
        tf2::Vector3 v3(-1.2, 0.35, 0);
        tf2::Vector3 v4(-1.2, -0.35, 0);

        // 将每个向量旋转相同的角度
        v1 = transform * v1;
        v2 = transform * v2;
        v3 = transform * v3;
        v4 = transform * v4;

        //轮子的姿态
        wheel_lf.pose.orientation.x =  dir_x;
        wheel_lf.pose.orientation.y = dir_y;
        wheel_lf.pose.orientation.z = dir_z;
        wheel_lf.pose.orientation.w = 1;
        //std::cout<<"四元数之和为："<<q_dir.w()*q_dir.w()+q_dir.z()*q_dir.z()+q_dir.y()*q_dir.y()+q_dir.x()*q_dir.x()<<std::endl;
        //轮子的尺寸
        wheel_lf.scale.x = 0.5;
        wheel_lf.scale.y = 0.5;
        wheel_lf.scale.z = 0.10;
        //轮子的颜色
        wheel_lf.color.a = 1.0;
        wheel_lf.color.r = 0.0;
        wheel_lf.color.g = 1.0;
        wheel_lf.color.b = 0.0;
        //轮子的位置相对与车辆位置的偏移量
        wheel_lf.pose.position.x = v1.getX() + Carstate.car_state.x;
        wheel_lf.pose.position.y = v1.getY() + Carstate.car_state.y;

        wheel_rf.pose.orientation.x = dir_x;
        wheel_rf.pose.orientation.y = dir_y;
        wheel_rf.pose.orientation.z = dir_z;
        wheel_rf.pose.orientation.w = 1;
        wheel_rf.scale.x = 0.5;
        wheel_rf.scale.y = 0.5;
        wheel_rf.scale.z = 0.10;
        wheel_rf.color.a = 1.0;
        wheel_rf.color.r = 0.0;
        wheel_rf.color.g = 1.0;
        wheel_rf.color.b = 0.0;
        wheel_rf.pose.position.x = v2.getX() + Carstate.car_state.x;
        wheel_rf.pose.position.y = v2.getY() + Carstate.car_state.y;

        wheel_lr.pose.orientation.x = dir_x;
        wheel_lr.pose.orientation.y = dir_y;
        wheel_lr.pose.orientation.z = dir_z;
        wheel_lr.pose.orientation.w = 1;
        wheel_lr.scale.x = 0.5;
        wheel_lr.scale.y = 0.5;
        wheel_lr.scale.z = 0.10;
        wheel_lr.color.a = 1.0;
        wheel_lr.color.r = 0.0;
        wheel_lr.color.g = 1.0;
        wheel_lr.color.b = 0.0;
        wheel_lr.pose.position.x = v3.getX() + Carstate.car_state.x;
        wheel_lr.pose.position.y = v3.getY() + Carstate.car_state.y;

        wheel_rr.pose.orientation.x = dir_x;
        wheel_rr.pose.orientation.y = dir_y;
        wheel_rr.pose.orientation.z = dir_z;
        wheel_rr.pose.orientation.w = 1;
        wheel_rr.scale.x = 0.5;
        wheel_rr.scale.y = 0.5;
        wheel_rr.scale.z = 0.10;
        wheel_rr.color.a = 1.0;
        wheel_rr.color.r = 0.0;
        wheel_rr.color.g = 1.0;
        wheel_rr.color.b = 0.0;
        wheel_rr.pose.position.x = v4.getX() + Carstate.car_state.x;
        wheel_rr.pose.position.y = v4.getY() + Carstate.car_state.y;

        //发布四个轮子标记
        wholeMarker_pub.publish(wheel_lf);
        wholeMarker_pub.publish(wheel_rf);
        wholeMarker_pub.publish(wheel_lr);
        wholeMarker_pub.publish(wheel_rr);
    }

    //锥筒可视化
    void Location::visCone(double x, double y, double z, int id)
    {
        visualization_msgs::Marker marker;                // 创建一个名为marker的visualization_msgs/Marker消息对象
        marker.header.frame_id = "velodyne";              // 设置marker消息的坐标系为map
        marker.lifetime = ros::Duration();                // 设置marker消息的持续时间为永久
        marker.header.stamp = ros::Time::now();           // 设置marker消息的时间戳为当前时间
        marker.ns = "my_namespace";                       // 设置marker消息的namespace为my_namespace，用于在RViz中区分不同的marker
        marker.id = id;                                   // 设置marker消息的id为0，用于在RViz中区分不同的marker
        marker.type = visualization_msgs::Marker::CYLINDER; // 设置marker消息的类型为柱体
        marker.action = visualization_msgs::Marker::ADD;  // 设置marker消息的操作类型为ADD，表示添加一个新的marker
        marker.pose.position.x = x;                       // 设置marker消息的位置x坐标为0
        marker.pose.position.y = y;                       // 设置marker消息的位置y坐标为0
        marker.pose.position.z = z;                       // 设置marker消息的位置z坐标为0
        marker.pose.orientation.x = 0.0;                  // 设置marker消息的方向x分量为0
        marker.pose.orientation.y = 0.0;                  // 设置marker消息的方向y分量为0
        marker.pose.orientation.z = 0.0;                  // 设置marker消息的方向z分量为0
        marker.pose.orientation.w = 1.0;                  // 设置marker消息的方向w分量为1，表示不进行旋转
        marker.scale.x = 0.3;                             // 设置marker消息的大小x分量为0.1
        marker.scale.y = 0.3;                             // 设置marker消息的大小y分量为0.1
        marker.scale.z = 0.7;                             // 设置marker消息的大小z分量为0.1
        marker.color.a = 1.0;                             // 设置marker消息的颜色透明度为1.0，表示完全不透明
        marker.color.r = 1.0;                             // 设置marker消息的颜色红分量为1.0，表示红色
        marker.color.g = 1.0;                             // 设置marker消息的颜色绿分量为0.0，表示不带绿色
        marker.color.b = 1.0;                             // 设置marker消息的颜色蓝分量为0.0，表示不带蓝色
        coneMarker_pub.publish(marker);                   // 发布marker消息到ROS系统中/coneMarker话题，用于在RViz中显示
    }

//存储相关测试文件
    //保存接收到的经纬度信息
    void Location::saveGPS(double lat,double lon,double alt)
    {
        std::stringstream ss;
            ss<< "lat = " <<lat<<"\t"<< "lon = " <<lon<<"\t"<< "alt = " <<alt<<std::endl;
        std::string str = ss.str();
        std::ofstream f;
        f.open("/home/mizorecanvas/tracking/src/testData/gps.txt",std::ios_base::app);
        if(f.fail())
        {
            std::cerr<<"Failed to open file: "<<std::strerror(errno)<<std::endl;
            return;
        }
        f<<str;
        f.close();
    }
    //保存角度变化信息
    void Location::saveAnglechange(double first,double now,double diff)
    {
        std::stringstream ss;
            ss<< "first = " <<first<<"\t"<< "now = " << now<<"\t"<< "diff" <<diff<<std::endl; //上一次方位角与方位角以及差值
        std::string str = ss.str();
        std::ofstream f;
        f.open("/home/mizorecanvas/tracking/src/testData/angleChange.txt",std::ios_base::app); 
        if (f.fail()) 
        { 
            std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl; 
            return; 
        }
        f << str;
        f.close();
    }
    //保存转换后的车辆坐标和前后轮轴中心坐标
    void Location::saveState(double x,double y,double yaw,double front_x,double front_y,double front_z,double rear_x,double rear_y,double rear_z)
    {
        std::stringstream ss;
            ss << x << "\t" << y << "\t" << yaw <<std::endl;
            ss << front_x << "\t"<< front_y << "\t" << front_z <<std::endl;
            ss << rear_x << "\t" << rear_y << "\t" << rear_z << "\n" <<std::endl;
        std::string str = ss.str();
        std::ofstream f;
        f.open("/home/mizorecanvas/tracking/src/testData/state.txt",std::ios_base::app);
        if(f.fail())
        {
            std::cerr<<"Failed to open file: "<<std::strerror(errno)<<std::endl;
            return;
        }
        f<<str;
        f.close();
    }
    //保存地图相关信息
    void Location::saveMap(common_msgs::HUAT_map &map)
    {
        std::stringstream ss;
        for (int i = 0; i < map.cone.size(); i++)
        {
            ss << map.cone[i].id << std::endl;
            ss << map.cone[i].position_global.x << "\t" << map.cone[i].position_global.y << std::endl;
            ss << map.cone[i].position_baseLink.x << "\t" << map.cone[i].position_baseLink.y << std::endl;
        }
        std::string str = ss.str();
        std::ofstream f;
        f.open("/home/mizorecanvas/tracking/src/testData/map.txt", std::ios_base::app);
        if (f.fail())
        {
            std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl;
            return;
        }
        f << str;
        f << "\n\n\n";
        f.close();
    }
    //保存基于全局坐标系下的锥筒信息
    void Location::saveCone(common_msgs::HUAT_cone &cone)
    {
        std::stringstream ss;
        ss << cone.id << "\t" << cone.position_global.x << "\t" << cone.position_global.y << std::endl;
        std::string str = ss.str();
        std::ofstream f;
        f.open("/home/mizorecanvas/tracking/src/testData/cone.txt", std::ios_base::app);
        if (f.fail())
        {
            std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl;
            return;
        }
        f << str;
        f.close();
    }
    //保存单个锥筒点的坐标信息
    void Location::savePoint(double x, double y, double z, int id)
    {
        std::stringstream ss;
        ss  << id << "\t" << x << "\t" << y << "\t" << z << std::endl;
        std::string str = ss.str();
        std::ofstream f;
        f.open("/home/mizorecanvas/tracking/src/testData/point.txt", std::ios_base::app);
        if (f.fail())
        {
            std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl;
            return;
        }
        f << str;
        f.close();
    }
    //保存车在ENU坐标系下的坐标
    void Location::saveCarstate(double x,double y)
    {
        ROS_WARN("SAVE");
        std::stringstream ss;
        ss << x << "\t" << y <<std::endl;
        std::string str = ss.str();
        std::ofstream f;
        f.open("/home/tb/tracking/src/testData/carstate.txt", std::ios_base::app);
        if (f.fail())
        {
            std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl;
            return;
        }
        f << str;
        f.close();
    }
    //保存车的线速度,角速度,线加速度
    void Location::saveAllvelocity(double V,double W,double A)
    {
        std::stringstream ss;
        ss << V << "\t" << W << "\t" << A <<std::endl;
        std::string str = ss.str();
        std::ofstream f;
        f.open("/home/mizorecanvas/tracking/src/testData/allvelocity.txt", std::ios_base::app);
        if (f.fail())
        {
            std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl;
            return;
        }
        f << str;
        f.close();
    }
    //保存车的东向,北向,地向速度
    void Location::saveVelocity(double E,double N,double G)
    {
        std::stringstream ss;
        ss << E << "\t" << N << "\t" << G <<std::endl;
        std::string str = ss.str();
        std::ofstream f;
        f.open("/home/mizorecanvas/tracking/src/testData/velocity.txt", std::ios_base::app);
        if (f.fail())
        {
            std::cerr << "Failed to open file: " << std::strerror(errno) << std::endl;
            return;
        }
        f << str;
        f.close();
    }
    //保存相关信息
    void Location::saveAcc(double x_acc,double y_acc,double z_acc)
    {
        std::stringstream ss;
        ss << x_acc << "\t" << y_acc << "\t" << z_acc <<std::endl;
        std::string str = ss.str();
        std::ofstream f;
        f.open("/home/mizorecanvas/tracking/src/testData/acc.txt", std::ios_base::app);
        if (f.fail())
        {
            return;
        }
        f << str;
        f.close();
    }
}
