//
// Created by wss on 7/1/17.
//

#ifndef PROJECT_MONITOR_WINDOWS_H
#define PROJECT_MONITOR_WINDOWS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cstdlib>            // For atoi()
#include <pthread.h>          // For POSIX threads
#include <boost/thread/thread.hpp>
#include <arpa/inet.h>

#include "vehicle_interface_core/udp_socket.h"
#include "vehicle_interface_ros/Node.h"


class Application
{

public:
    Application(int argc, char** argv);
    ~Application();

private:
    void start(void);

    void processPendingDatagramsIns( void);
    void processPendingDatagramsVehicle( void);

    void vehicleSendUdp();

private:
    UserNode userNode;
    boost::thread *insThread;
    boost::thread *vehicleThread;

    UDPSocket *insUdpSocket;
    UDPSocket *vehicleUdpSocket;

    string insUrl = "127.0.0.1";
    string vehicleUrl = "192.168.1.240";
    uint16_t insPort;
    uint16_t vehiclePort;

    uint8_t ins_rx_msg[INS_INFO_LENGTH];
    uint8_t vehicle_rx_msg[VEHICLE_INFO_LENGHT];

    uint8_t vehicle_tx_cmd_msg[VEHICLE_CMD_LENGHT];
    bool is_running;
};
#endif
