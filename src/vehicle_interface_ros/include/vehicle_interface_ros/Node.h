//
// Created by wss on 7/1/17.
//

#ifndef PROJECT_MONITOR_NODE_H
#define PROJECT_MONITOR_NODE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <ros/network.h>
#include <boost/thread/thread.hpp>
#include <boost/functional.hpp>

#include "autodrive_msgs/HUAT_VehicleStatus.h"
#include "autodrive_msgs/HUAT_VehicleCmd.h"
#include "autodrive_msgs/HUAT_InsP2.h"
#include "autodrive_msgs/topic_contract.hpp"

#define MSG_TOPIC_GLOBALPOSE               autodrive_msgs::topic_contract::kInsMsg
#define MSG_TOPIC_VEHICLE_STATUS           autodrive_msgs::topic_contract::kVehicleStatus
#define MSG_TOPIC_VEHICLE_CMD              autodrive_msgs::topic_contract::kVehicleCmd

#define IMU_LEN                     100

#define INS_INFO_LENGTH             69
#define VEHICLE_INFO_LENGHT         21
#define VEHICLE_CMD_LENGHT          12

typedef union doublebyte
{
    double mdouble;
    uint8_t ch[8];
} doublebyte;

typedef union floatbyte
{
    float mfloat;
    uint8_t ch[4];
} floatbyte;


typedef struct {
    uint16_t Week;
    float Time;
    float Heading;
    float Pitch;
    float Roll;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    double Lat;
    double Lon;
    float Altitude;
    float Ve;
    float Vn;
    float Vd;
    float Base;
    uint8_t NSV1;
    uint8_t NSV2;
    uint8_t Status;
    uint8_t Age;
    uint8_t War;
    //uint8_t CHeck;
} imu_msg;

typedef struct {
	uint16_t sync :16;
	uint8_t length :8;
	uint8_t reserved :8;
	uint16_t week :16;
	uint32_t time :32;
	uint8_t N :8;
	double latitude ;
	double longitude ;
	float altitude ;
	float nor_speed ;
	float eas_speed ;
	float down_speed ;
	float roll ;
	float pitch ;
	float heading ;
	int16_t nor_accelerate :16;
	int16_t eas_accelerate :16;
	int16_t down_accelerate :16;
	int16_t roll_speed :16;
	int16_t pitch_speed :16;
	int16_t heading_speed :16;
	uint8_t flag :8;
	uint8_t checksum :8;
} Packet;

typedef struct IPC2VCU{
    uint8_t head1;
    uint8_t head2;
    uint8_t length;
    uint8_t steering;
    uint8_t brake_force;
    uint8_t pedal_ratio;
    uint8_t gear_position;
    uint8_t working_mode;
    uint8_t racing_num;
    uint8_t racing_status;
    uint16_t checksum;
} ipc2vcu;

typedef struct VCU2IPC{
    uint8_t head1;
    uint8_t head2;
    uint8_t length;
    uint8_t steering;
    uint8_t brake_status;
    uint8_t pedal_ratio;
    uint8_t gear_position;
    uint16_t speed_left_front;
    uint16_t speed_right_front;
    uint16_t speed_left_rear;
    uint16_t speed_right_rear;
    uint8_t command;
    uint8_t work_mode;
    uint8_t racing_num;
    uint8_t fault_type;
    uint16_t checksum;
}vcu2ipc;

typedef std::function<void(void)> udpCallback;

class UserNode
{
public:
    UserNode(int argc, char **pArgv);
    ~UserNode();
    void init();
    bool getReady();

    void publishVehicle(void *data);
    void publishIns(void *data);
    uint8_t getVehicleUdpMessage(void* data);
    void setVehicleCallBack(udpCallback fun);

private:
	void run();

	void useVehicleCallBack();

	void Imu_Trans();

    void recv_msg_vehcileCMD_callack(const autodrive_msgs::HUAT_VehicleCmd::ConstPtr &pVehicleMsg);

private:
    int m_Init_argc;
    char** m_pInit_argv;

    bool rosRunning;
    bool m_bExit;

    boost::thread *p_vehicle_ros_thread;


    autodrive_msgs::HUAT_InsP2 global_insMsg;

    autodrive_msgs::HUAT_VehicleCmd vehicle_cmd_msg;
    autodrive_msgs::HUAT_VehicleStatus vehicle_status_msg;

    ros::Publisher  ros_pub_ins_info;
    ros::Publisher  ros_pub_vehicle_info;

    ros::Subscriber ros_recv_vehicle_msg;

    uint8_t vehicle_tx_msg[VEHICLE_CMD_LENGHT];

    udpCallback vehicleUdpCallback;
    uint8_t imu_buffer[IMU_LEN];
    imu_msg imu_data;
	Packet imuPacket;

    vcu2ipc vcu2ipc_msg;
};


#endif //PROJECT_MONITOR_NODE_H
