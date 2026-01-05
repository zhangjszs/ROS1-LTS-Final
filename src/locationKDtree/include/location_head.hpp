#ifndef LOCATION_HPP
#define LOCATION_HPP

#include "string"
#include "iostream"
#include "vector"
#include "fstream"
#include "utility"
#include "map"
#include "sstream"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "flann/flann.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include "common_msgs/HUAT_ASENSING.h"
#include "common_msgs/HUAT_Carstate.h"
//#include "common_msgs/HUAT_A_cone.h"
#include "common_msgs/HUAT_cone.h"
#include "common_msgs/Cone.h"
#include "common_msgs/HUAT_map.h"

#define PII 3.14159265358979

//用于简单转化为弧度制
#ifndef DEG2RAD
#define DEG2RAD(deg) ( (deg) * M_PI / 180.0 )
#endif

namespace coordinate
{
    class Location
    {
        public:
        Location(ros::NodeHandle &nh);
        void loadParameters();

        void saveGPS(double lat,double lon,double alt);
        void saveState(double x,double y,double yaw,double front_x,double front_y,double front_z,double rear_x,double rear_y,double rear_z);
        void saveAnglechange(double first,double now,double diff);
        void saveMap(common_msgs::HUAT_map &map);
        void saveCone(common_msgs::HUAT_cone &cone);
        void savePoint(double x,double y,double z,int id);
        void saveCarstate(double x,double y);
        void saveAllvelocity(double V,double W,double A);
        void saveVelocity(double E,double N,double G);
        void saveAcc(double x_acc,double y_acc,double z_acc);

        private:
        ros::Subscriber INS_sub;//订阅惯导消息
        ros::Subscriber cone_sub;//订阅锥筒消息

        ros::Publisher carState_pub;//发布车辆在全局坐标系下的坐标
        ros::Publisher map_pub;//发布所有锥筒的id和它们在全局和车身坐标系下的坐标
        ros::Publisher globalMap_pub;//发布锥筒点云信息
        //可视化
        ros::Publisher carMarker_pub;
        ros::Publisher wholeMarker_pub;
        ros::Publisher coneMarker_pub;

        common_msgs::HUAT_Carstate Carstate;//存储车辆位置的相关信息
        common_msgs::HUAT_ASENSING Mimu;
        common_msgs::HUAT_ASENSING Mins;//存储INS相关信息
        common_msgs::HUAT_cone Ycone;//存储当时锥筒的信息
        common_msgs::HUAT_map Ymap;//存储所有锥筒的信息
        
        ros::NodeHandle nh_;
        std::string subTopic_; 
        int id = 0;
        double enu_xyz[3];//车在ENU坐标系下的坐标
        double front_wheel[3];//前轮轴中心在ENU坐标系下的坐标
        double rear_wheel[3];//后轮轴中心在ENU坐标系下的坐标
        double first_lat,first_lon,first_alt;//第一次的经纬度以及海拔
        double standardAzimuth;
        double frontToIMUdistanceX_,frontToIMUdistanceY_,frontToIMUdistanceZ_,rearToIMUdistanceX_,rearToIMUdistanceY_,rearToIMUdistanceZ_,lidarToIMUDist_;//参数服务器获取的消息
        double tfRoll,tfPitch;//角度转换为弧度制
        //车辆前进方向向量
        double dir_x = 0.0;
        double dir_y = 0.0;
        double dir_z = 0.0;
    
        bool isfirstINSreceived = false;//判断是否是第一次接收惯导消息
        bool firstConeMsg = false;//判断是否是第一次接收锥筒消息
        bool publish_visualization_ = true;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        std::vector<int> point_ids;
        
        void GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3]);
        int getNewId();
        void calcVehicleDirection(double roll, double pitch, double yaw, double &x, double &y, double &z); 

        void doINSMsg(const common_msgs::HUAT_ASENSING::ConstPtr& msgs);
        void doConeMsg(const common_msgs::Cone::ConstPtr& msgs);
        
        void visWhole();
        void visCone(double x,double y,double z,int id);
        void visCar();
    };
}
#endif
