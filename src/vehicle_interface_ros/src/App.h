//
// Created by wss on 7/1/17.
//

#ifndef VEHICLE_INTERFACE_ROS_APP_H
#define VEHICLE_INTERFACE_ROS_APP_H

#include "vehicle_interface_core/udp_socket.h"
#include "vehicle_interface_ros/Node.h"

#include <cstdlib>  // For atoi()
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

#include <arpa/inet.h>
#include <boost/thread/thread.hpp>
#include <pthread.h>  // For POSIX threads

class Application {
 public:
  Application(int argc, char** argv);
  ~Application();

 private:
  void start(void);

  void processPendingDatagramsIns(void);
  void processPendingDatagramsVehicle(void);

  void vehicleSendUdp();

 private:
  UserNode userNode;
  std::unique_ptr<boost::thread> insThread;
  std::unique_ptr<boost::thread> vehicleThread;

  std::unique_ptr<UDPSocket> insUdpSocket;
  std::unique_ptr<UDPSocket> vehicleUdpSocket;

  std::string insUrl = "127.0.0.1";
  std::string vehicleUrl = "192.168.1.240";
  uint16_t insPort;
  uint16_t vehiclePort;

  uint8_t ins_rx_msg[INS_INFO_LENGTH];
  uint8_t vehicle_rx_msg[VEHICLE_INFO_LENGTH];

  uint8_t vehicle_tx_cmd_msg[VEHICLE_CMD_LENGTH];
  bool is_running;
  bool verify_checksum_{false};

  // B11: verify sum-of-bytes checksum on received vehicle frame
  static bool VerifyVehicleChecksum(const uint8_t* buf, int len);
};
#endif
