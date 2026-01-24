#include "App.h"

Application::Application(int argc, char **argv)
        : userNode(argc, argv)
{
    start();

    vehicleThread = new boost::thread(boost::bind(&Application::processPendingDatagramsVehicle, this));
    insThread = new boost::thread(boost::bind(&Application::processPendingDatagramsIns, this));

    userNode.setVehicleCallBack(std::bind(&Application::vehicleSendUdp, this));

    userNode.init();
}

Application::~Application()
{
    vehicleUdpSocket->disconnect();
    insUdpSocket->disconnect();
    vehicleThread->join();
    insThread->join();
}

void Application::start(void) {

    vehiclePort = 2040;
    insPort = 12340;

    vehicleUdpSocket= new UDPSocket(vehiclePort);
    insUdpSocket = new UDPSocket(insPort);
}

void Application::processPendingDatagramsVehicle( void){
    int iLen;
    uint8_t vehicle_buf[1024];

    std::cout << "vehicle Thread"<< std::endl;

    while(true){
        iLen = vehicleUdpSocket->recvFrom(vehicle_buf, VEHICLE_INFO_LENGHT, vehicleUrl, vehiclePort);

        if((0xAA == vehicle_buf[0]) && (0x55 == vehicle_buf[1]) &&(iLen == VEHICLE_INFO_LENGHT) &&(userNode.getReady()))
        {
            memcpy(vehicle_rx_msg,vehicle_buf,VEHICLE_INFO_LENGHT);
			userNode.publishVehicle((void*) vehicle_rx_msg);
			std::cout << ".";
        }
    }
}

void Application::processPendingDatagramsIns( void) {
    int iLen;
    uint8_t ins_buf[1024];

    std::cout << "ins Thread"<< std::endl;

    while (true) {
        iLen = insUdpSocket->recvFrom(ins_buf, INS_INFO_LENGTH, insUrl, insPort);
        std::cout<<"iLen:"<<iLen<<" "<<INS_INFO_LENGTH<<std::endl;
        if ((iLen == INS_INFO_LENGTH) &&(userNode.getReady())) {
            memcpy(ins_rx_msg, ins_buf, INS_INFO_LENGTH);
			userNode.publishIns((void*) ins_rx_msg);
        }
    }
}

void Application::vehicleSendUdp(){
    uint8_t msgLength;
    msgLength = userNode.getVehicleUdpMessage((void*)vehicle_tx_cmd_msg);
    std::cout <<"sending"<<std::endl;
    vehicleUdpSocket->sendTo((void*)vehicle_tx_cmd_msg, msgLength, vehicleUrl, vehiclePort);
}

