#include "test.hpp"
#include "line.hpp"
#include "skip.hpp"
#include "high.hpp"
#include <stdlib.h>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    
    // ofstream ofs;
    // ofs.open("/home/tb/huat2025/src/control/doc/my_cmd.txt", ios::out);
    // ofs.close();

    ros::init(argc, argv, "control_new");

    ros::NodeHandle nh;
    common_msgs::HUAT_VehcileCmd cmd;
    ros::Duration(1).sleep();
    ros::Rate rate(10);
    control::Car *my_car;

    int num = 0;
    std::ifstream ifs;
    while(!num)
    {
        std::string home_path = getenv("HOME");
        std::string cmd_path = home_path + "/autoStartGkj/command";
        ifs.open(cmd_path.c_str(), std::ios::in);
        if (!ifs.is_open())
        {
            std::cout << "文件打开失败" << std::endl;
            return 0;
        }
        num = (char)ifs.get() - '0';
        ifs.close();
    }

    if (num == 1)
        my_car = new control::Test;
    else if (num == 2)
        my_car = new control::Line;
    else if (num == 3)
        my_car = new control::Skip;
    else
        my_car = new control::High;

    my_car->num = num;
    while (ros::ok())
    {
        ros::spinOnce();
        my_car->control_cmd(cmd);
        rate.sleep();
    }

    delete my_car;
    return 0;
}
