#include "ros/ros.h"
#include "common_msgs/Cone.h"

void coneMsgCallback(const common_msgs::Cone::ConstPtr &msgs)
{
    int size = (msgs->confidence).size();

    std::vector<float> confs = msgs->confidence;

    std::cout << "conf array: size:" << size << std::endl;

    for (float cf : confs)
    {
        std::cout << cf << " ";
    }
    std::cout << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_conf");

    ros::NodeHandle nh;
    ros::Subscriber cone_msg_sub = nh.subscribe("/cone_position", 1, coneMsgCallback);

    ros::spin();
}