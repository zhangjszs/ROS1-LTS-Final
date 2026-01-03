/*
Author: Adams
Time: 22/3/25
*/

#include <sstream>
#include "ros/ros.h"                 //引入ros头文件
#include<iostream> //C++标准输入输出库
#include <fstream>
#include<string> //C++标准输入输出库
#include <common_msgs/vehicle_status.h>


using namespace std;

// outfile.precision(10);
void racingNumCmdCallback(const common_msgs::vehicle_status v_status){
    int head1 = int (v_status.head1);
    int head2 = int (v_status.head2);
    int length= int (v_status.length);
    int steering= int (v_status.steering);
    int brake_status= int (v_status.brake_status);
    int pedal_ratio= int (v_status.pedal_ratio);
    int gear_position= int (v_status.gear_position);
    int speed_left_front= int (v_status.speed_left_front);
    int speed_right_front= int (v_status.speed_right_front);
    int speed_left_rear= int (v_status.speed_left_rear);
    int speed_right_rear= int (v_status.speed_right_rear);
    int command= int (v_status.command);
    int work_mode= int (v_status.work_mode);
    int racing_num= int (v_status.racing_num);
    int fault_type= int (v_status.fault_type);
    int checksum= int (v_status.checksum);

	// cout << " head1:"<< head1<<endl;
	// cout << " head2:"<< head2<<endl;
	// cout << " length:"<< length<<endl;
	// cout << " steering:"<< steering<<endl;
	// cout << " brake_status:"<< brake_status<<endl;
	// cout << " pedal_ratio:"<< pedal_ratio;
	// cout << " gear_position:"<< gear_position;
	// cout << " speed_left_front:"<< speed_left_front;
	// cout << " speed_right_front:"<< speed_right_front;
	// cout << " speed_left_rear:"<< speed_left_rear;
	// cout << " speed_right_rear:"<< speed_right_rear;
	// cout << " command:"<< command;
	cout << " work_mode:"<< work_mode<<endl;
	cout << " racing_num:"<< racing_num<<endl;
	// cout << " fault_type:"<< fault_type;
	// cout << " checksum:"<< checksum;

    // ofstream outfile("~/autoStartGkj/command",ofstream::out | ofstream::trunc);
    ofstream outfile("/home/tb/autoStartGkj/command",ofstream::trunc);
    // if(racingNum !=0){
    if (outfile.is_open()){
        cout<<"File Open Succ!"<<endl;
        outfile << racing_num;
        
        // cout<<"."<<endl;
        outfile.close();
        if(racing_num!=0)
            exit(0);
    }else{
        cout<<"File Open Failed!"<<endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "racingNumCmd");
    ros::NodeHandle n;
    ros::Subscriber ins_sub = n.subscribe("vehicleStatusMsg", 1, racingNumCmdCallback);
    ros::spin();
    return 0;
}

// 该程序的目的是监听车辆状态消息,并在接收到消息时将赛车编号写入指定文件
// 如果赛车编号不为0,程序将退出
