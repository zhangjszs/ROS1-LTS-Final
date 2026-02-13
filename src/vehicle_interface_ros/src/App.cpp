#include "App.h"

Application::Application(int argc, char **argv)
        : userNode(argc, argv), is_running(true)
{
    start();

    vehicleThread = std::make_unique<boost::thread>(boost::bind(&Application::processPendingDatagramsVehicle, this));
    insThread = std::make_unique<boost::thread>(boost::bind(&Application::processPendingDatagramsIns, this));

    userNode.setVehicleCallBack(std::bind(&Application::vehicleSendUdp, this));

    userNode.init();
}

Application::~Application()
{
    is_running = false;
    vehicleUdpSocket->shutdownSocket();
    insUdpSocket->shutdownSocket();
    vehicleThread->join();
    insThread->join();
}

void Application::start(void) {

    // Read ports from ROS parameter server; fall back to hardcoded defaults.
    ros::NodeHandle pnh("~");
    int vp = 2040, ip = 12340;
    pnh.param("vehicle_port", vp, 2040);
    pnh.param("ins_port", ip, 12340);
    vehiclePort = static_cast<uint16_t>(vp);
    insPort = static_cast<uint16_t>(ip);

    vehicleUdpSocket = std::make_unique<UDPSocket>(vehiclePort);
    insUdpSocket = std::make_unique<UDPSocket>(insPort);
}

void Application::processPendingDatagramsVehicle( void){
    int iLen;
    uint8_t vehicle_buf[1024];

    while(is_running){
        iLen = vehicleUdpSocket->recvFrom(vehicle_buf, VEHICLE_INFO_LENGHT, vehicleUrl, vehiclePort);
        if (iLen < 0) break;

        if((0xAA == vehicle_buf[0]) && (0x55 == vehicle_buf[1]) &&(iLen == VEHICLE_INFO_LENGHT) &&(userNode.getReady()))
        {
            memcpy(vehicle_rx_msg,vehicle_buf,VEHICLE_INFO_LENGHT);
			userNode.publishVehicle((void*) vehicle_rx_msg);
        }
    }
}

void Application::processPendingDatagramsIns( void) {
    int iLen;
    uint8_t ins_buf[1024];

    while (is_running) {
        iLen = insUdpSocket->recvFrom(ins_buf, INS_INFO_LENGTH, insUrl, insPort);
        if (iLen < 0) break;
        if ((iLen == INS_INFO_LENGTH) &&(userNode.getReady())) {
            memcpy(ins_rx_msg, ins_buf, INS_INFO_LENGTH);
			userNode.publishIns((void*) ins_rx_msg);
        }
    }
}

void Application::vehicleSendUdp(){
    uint8_t msgLength;
    msgLength = userNode.getVehicleUdpMessage((void*)vehicle_tx_cmd_msg);
    vehicleUdpSocket->sendTo((void*)vehicle_tx_cmd_msg, msgLength, vehicleUrl, vehiclePort);
}
