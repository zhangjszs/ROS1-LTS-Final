#include "location_head.hpp"

//using namespace coordinate;

int main(int argc ,char* argv[])
{
    
    ros::init(argc, argv, "location_kdtree");
    ros::NodeHandle nh;
    
    coordinate::Location loc(nh);
    
    ros::Rate rate(50);

    while (ros::ok())
    {  
        ros::spinOnce();  
        //ROS_INFO("Location node is running...");
        
        rate.sleep();
    }

    return 0; 
}
