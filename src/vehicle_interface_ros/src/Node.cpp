//
// Created by wss on 7/1/17.
//

#include "vehicle_interface_ros/Node.h"

UserNode::UserNode(int argc, char **pArgv)
        :m_Init_argc(argc),
         m_pInit_argv(pArgv)
{
    rosRunning = false;
    m_bExit = false;
    global_insMsg.header.seq = 0;
}
UserNode::~UserNode() {
    m_bExit = true;
    if(p_vehicle_ros_thread)
    {
        p_vehicle_ros_thread->join();
        delete p_vehicle_ros_thread;
        p_vehicle_ros_thread = nullptr;
    }
    if (ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }
}
/**
 *  ros function
 * */
void UserNode::init()
{
    std::string name;

    ros::init(m_Init_argc, m_pInit_argv, "vehicle_interface");

    if (!ros::master::check())
        return;
    // else{
    //     ROS_INFO_STREAM("")
    // }

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;

    name = ros::this_node::getName();

    ros_pub_ins_info = nh.advertise < autodrive_msgs::HUAT_InsP2> (MSG_TOPIC_GLOBALPOSE, 1);
    ros_pub_vehicle_info = nh.advertise < autodrive_msgs::HUAT_VehicleStatus> (MSG_TOPIC_VEHICLE_STATUS, 1);

    ros_recv_vehicle_msg =   nh.subscribe <autodrive_msgs::HUAT_VehicleCmd> \
            (MSG_TOPIC_VEHICLE_CMD, 1, &UserNode::recv_msg_vehcileCMD_callack, this);

    p_vehicle_ros_thread = new boost::thread(boost::bind(&UserNode::run, this));


}

void UserNode::run()
{
    // if(!ros::ok()){
    //     ROS_INFO_STREAM("roscore not open!");
    // }
    ros::Rate loop_rate(100);
    ROS_INFO_STREAM("rosrunning...");
    rosRunning = true;
    while (ros::ok() && !m_bExit)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool UserNode::getReady(){
    return rosRunning;
}

void UserNode::publishVehicle(void *data)
{
    unsigned char* buffer = (unsigned char*)data;

    vcu2ipc_msg.head1 = buffer[0];
    vcu2ipc_msg.head2 = buffer[1];
    vcu2ipc_msg.length = buffer[2];
    vcu2ipc_msg.steering = buffer[3];
    vcu2ipc_msg.brake_status = buffer[4];
    vcu2ipc_msg.pedal_ratio = buffer[5];
    vcu2ipc_msg.gear_position = buffer[6];
    vcu2ipc_msg.speed_left_front = buffer[7]* 256 + buffer[8];
    vcu2ipc_msg.speed_right_front = buffer[9]* 256 + buffer[10];
    vcu2ipc_msg.speed_left_rear = buffer[11]* 256 + buffer[12];
    vcu2ipc_msg.speed_right_rear = buffer[13]* 256 + buffer[14];
    vcu2ipc_msg.command = buffer[15];
    vcu2ipc_msg.work_mode = buffer[16];
    vcu2ipc_msg.racing_num = buffer[17];
    vcu2ipc_msg.fault_type = buffer[18];
    vcu2ipc_msg.checksum = buffer[19]*256 + buffer[20];


    vehicle_status_msg.head1 = vcu2ipc_msg.head1;
    vehicle_status_msg.head2 = vcu2ipc_msg.head2;
    vehicle_status_msg.length = vcu2ipc_msg.length;
    vehicle_status_msg.steering = vcu2ipc_msg.steering;
    vehicle_status_msg.brake_status = vcu2ipc_msg.brake_status;
    vehicle_status_msg.pedal_ratio = vcu2ipc_msg.pedal_ratio;
    vehicle_status_msg.gear_position = vcu2ipc_msg.gear_position;
    vehicle_status_msg.speed_left_front = vcu2ipc_msg.speed_left_front;
    vehicle_status_msg.speed_left_rear = vcu2ipc_msg.speed_left_rear;
    vehicle_status_msg.speed_right_front = vcu2ipc_msg.speed_right_front;
    vehicle_status_msg.speed_right_rear = vcu2ipc_msg.speed_right_rear;
    vehicle_status_msg.command = vcu2ipc_msg.command;
    vehicle_status_msg.work_mode = vcu2ipc_msg.work_mode;
    vehicle_status_msg.racing_num = vcu2ipc_msg.racing_num;
    vehicle_status_msg.fault_type = vcu2ipc_msg.fault_type;
    std::cout<<"Interface vehicle steering:"<< (int)(vcu2ipc_msg.pedal_ratio)<<std::endl<<(int)(vehicle_status_msg.steering)<<std::endl;

    ros_pub_vehicle_info.publish(vehicle_status_msg);
}

void UserNode::Imu_Trans(){

    uint16_t u16Byte;
    uint32_t u32Byte;
    uint8_t u8Byte;
    double d64Byte;
    float f32Byte;
    int16_t s16Byte;

    memcpy(&u8Byte, imu_buffer + 2, 1);
    imuPacket.length = u8Byte;

    memcpy(&u16Byte, imu_buffer + 4, 2);
    imuPacket.week = u16Byte;

    memcpy(&u32Byte, imu_buffer + 6, 4);
    imuPacket.time = u32Byte;

    memcpy(&u8Byte, imu_buffer + 10, 1);
    imuPacket.N = u8Byte;

    memcpy(&d64Byte, imu_buffer + 11, 8);
    imuPacket.latitude = d64Byte;

    memcpy(&d64Byte, imu_buffer + 19, 8);
    imuPacket.longitude = d64Byte;

    memcpy(&f32Byte, imu_buffer + 27, 4);
    imuPacket.altitude = f32Byte;

    memcpy(&f32Byte, imu_buffer + 31, 4);
    imuPacket.nor_speed = f32Byte;

    memcpy(&f32Byte, imu_buffer + 35, 4);
    imuPacket.eas_speed = f32Byte;

    memcpy(&f32Byte, imu_buffer + 39, 4);
    imuPacket.down_speed = f32Byte;

    memcpy(&f32Byte, imu_buffer + 43, 4);
    imuPacket.roll = f32Byte;

    memcpy(&f32Byte, imu_buffer + 47, 4);
    imuPacket.pitch = f32Byte;

    memcpy(&f32Byte, imu_buffer + 51, 4);
    imuPacket.heading = f32Byte;

    memcpy(&s16Byte, imu_buffer + 55, 2);
    imuPacket.nor_accelerate = s16Byte;

    memcpy(&s16Byte, imu_buffer + 57, 2);
    imuPacket.eas_accelerate = s16Byte;

    memcpy(&s16Byte, imu_buffer + 59, 2);
    imuPacket.down_accelerate = s16Byte;

    memcpy(&s16Byte, imu_buffer + 61, 2);
    imuPacket.roll_speed = s16Byte;

    memcpy(&s16Byte, imu_buffer + 63, 2);
    imuPacket.pitch_speed = s16Byte;

    memcpy(&s16Byte, imu_buffer + 65, 2);
    imuPacket.heading_speed = s16Byte;

    memcpy(&u8Byte, imu_buffer + 67, 1);
    imuPacket.flag = u8Byte;

    memcpy(&u8Byte, imu_buffer + 68, 1);
    imuPacket.checksum = u8Byte;

    imu_data.Heading= imuPacket.heading;
    imu_data.Pitch= imuPacket.pitch;

    imu_data.Lat=imuPacket.latitude;
    imu_data.Lon=imuPacket.longitude;
    // std::cout<<"Lat:"<<imu_data.Lat;
    // std::cout<<"Lon:"<<imu_data.Lon;
    imu_data.gyro_x = imuPacket.roll_speed;
    imu_data.gyro_y = imuPacket.pitch_speed;
    imu_data.gyro_z = imuPacket.heading_speed;
    imu_data.Ve=imuPacket.eas_speed;
    imu_data.Vn=imuPacket.nor_speed;
    imu_data.Vu=imuPacket.down_speed;
}

void UserNode::publishIns(void *data){
    uint8_t* ins_data = (uint8_t*)data;

    if(ins_data != nullptr) {
        uint8_t i;
        for (i = 0; i < INS_INFO_LENGTH; i++,ins_data++) {
            imu_buffer[i] = *ins_data;
        }

        Imu_Trans();

        global_insMsg.header.frame_id="imu";
        global_insMsg.header.stamp=ros::Time::now();
        global_insMsg.header.seq++;
        global_insMsg.Heading =imu_data.Heading;
        global_insMsg.Pitch =imu_data.Pitch;
        global_insMsg.gyro_x = imu_data.gyro_x;
        global_insMsg.gyro_y = imu_data.gyro_y;
        global_insMsg.gyro_z = imu_data.gyro_z;
        global_insMsg.Lat =imu_data.Lat;
        global_insMsg.Lon =imu_data.Lon;

        global_insMsg.Ve =imu_data.Ve;
        global_insMsg.Vn =imu_data.Vn;
        global_insMsg.Vu =imu_data.Vu;
        // std::cout<<"Received Lat Lon:"<<global_insMsg.Lat<<std::endl;
        ros_pub_ins_info.publish(global_insMsg);
    }

}


void UserNode::recv_msg_vehcileCMD_callack(const autodrive_msgs::HUAT_VehicleCmd::ConstPtr &pVehicleCMD) {

    vehicle_cmd_msg = *pVehicleCMD;

    /**/

    vehicle_tx_msg[0] = 0xAA;
    vehicle_tx_msg[1] = 0x55;
    vehicle_tx_msg[2] = VEHICLE_CMD_LENGHT;
    vehicle_tx_msg[3] = vehicle_cmd_msg.steering;
    vehicle_tx_msg[4] = vehicle_cmd_msg.brake_force;
    vehicle_tx_msg[5] = vehicle_cmd_msg.pedal_ratio;
    vehicle_tx_msg[6] = vehicle_cmd_msg.gear_position;
    vehicle_tx_msg[7] = vehicle_cmd_msg.working_mode;
    vehicle_tx_msg[8] = vehicle_cmd_msg.racing_num;
    vehicle_tx_msg[9] = vehicle_cmd_msg.racing_status;
    vehicle_tx_msg[10] = 0x00;
    vehicle_tx_msg[11] = 0x00;
    std::cout<<"STEERING:"<<(int)vehicle_cmd_msg.steering<<(int)vehicle_cmd_msg.pedal_ratio<<std::endl;
    if((int)vehicle_cmd_msg.brake_force == 80){
        std::cout<<"brake!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    }
    // std::cout<<"received: "<<(int)vehicle_cmd_msg.steering<<std::endl;
    // std::cout<<"received: "<<(int)vehicle_cmd_msg.brake_force<<std::endl;
    useVehicleCallBack();
}

uint8_t UserNode::getVehicleUdpMessage(void* data){

    memcpy(data,vehicle_tx_msg,VEHICLE_CMD_LENGHT);

    return VEHICLE_CMD_LENGHT;
}

void UserNode::setVehicleCallBack(udpCallback fun){
	vehicleUdpCallback =fun;
}
/**
 * set udp callback.
 **/
void UserNode::useVehicleCallBack(){
    vehicleUdpCallback();
}
