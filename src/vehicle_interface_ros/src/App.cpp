#include "App.h"

#include <cerrno>

// B11: sum-of-bytes checksum verification for vehicle RX frames
bool Application::VerifyVehicleChecksum(const uint8_t *buf, int len)
{
    if (len < 3) return false;
    // Checksum is last 2 bytes (little-endian uint16), computed over preceding bytes
    const int payload_len = len - 2;
    uint16_t sum = 0;
    for (int i = 0; i < payload_len; ++i)
    {
        sum += buf[i];
    }
    uint16_t received = static_cast<uint16_t>(buf[payload_len]) |
                        (static_cast<uint16_t>(buf[payload_len + 1]) << 8);
    return sum == received;
}

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
    pnh.param("verify_checksum", verify_checksum_, false);
    vehiclePort = static_cast<uint16_t>(vp);
    insPort = static_cast<uint16_t>(ip);

    vehicleUdpSocket = std::make_unique<UDPSocket>(vehiclePort);
    insUdpSocket = std::make_unique<UDPSocket>(insPort);

    // B17: set 500ms receive timeout to avoid blocking forever
    vehicleUdpSocket->setReceiveTimeout(500);
    insUdpSocket->setReceiveTimeout(500);
}

void Application::processPendingDatagramsVehicle( void){
    int iLen;
    uint8_t vehicle_buf[1024];

    while(is_running){
        iLen = vehicleUdpSocket->recvFrom(vehicle_buf, VEHICLE_INFO_LENGHT, vehicleUrl, vehiclePort);
        if (iLen < 0) {
            // B17: timeout (EAGAIN/EWOULDBLOCK) — retry instead of exit
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            break;
        }

        if((0xAA == vehicle_buf[0]) && (0x55 == vehicle_buf[1]) &&(iLen == VEHICLE_INFO_LENGHT) &&(userNode.getReady()))
        {
            // B11: optional checksum verification
            if (verify_checksum_ && !VerifyVehicleChecksum(vehicle_buf, iLen)) {
                ROS_WARN_THROTTLE(1.0, "[VehicleInterface] RX vehicle checksum mismatch, dropping frame");
                continue;
            }
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
        if (iLen < 0) {
            // B17: timeout (EAGAIN/EWOULDBLOCK) — retry instead of exit
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            break;
        }
        if ((iLen == INS_INFO_LENGTH) &&(userNode.getReady())) {
            memcpy(ins_rx_msg, ins_buf, INS_INFO_LENGTH);
			userNode.publishIns((void*) ins_rx_msg);
        }
    }
}

void Application::vehicleSendUdp(){
    uint8_t msgLength;
    msgLength = userNode.getVehicleUdpMessage((void*)vehicle_tx_cmd_msg);

    // B10: validate command frame before sending
    if (msgLength != VEHICLE_CMD_LENGHT) {
        ROS_ERROR("[VehicleInterface] TX frame length mismatch: %d (expected %d), dropping",
                  msgLength, VEHICLE_CMD_LENGHT);
        return;
    }
    if (vehicle_tx_cmd_msg[0] != 0xAA || vehicle_tx_cmd_msg[1] != 0x55) {
        ROS_ERROR("[VehicleInterface] TX frame sync bytes invalid: 0x%02X 0x%02X, dropping",
                  vehicle_tx_cmd_msg[0], vehicle_tx_cmd_msg[1]);
        return;
    }

    vehicleUdpSocket->sendTo((void*)vehicle_tx_cmd_msg, msgLength, vehicleUrl, vehiclePort);
}
