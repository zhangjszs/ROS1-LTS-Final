//
// Created by wss on 9/9/17.
//

#include "vehicle_interface_core/udp_socket.h"

#include <sys/socket.h>      // For socket(), connect(), send(), and recv()
#include <netdb.h>           // For gethostbyname()
#include <arpa/inet.h>       // For inet_addr()
#include <unistd.h>          // For close()
#include <netinet/in.h>      // For sockaddr_in
typedef void raw_type;       // Type used for raw data on this platform

#include <cerrno>             // For errno
#include <iostream>


using namespace std;


// Function to fill in address structure given an address and port
static void fillAddr(const string &address, unsigned short port,
                     sockaddr_in &addr) {
    memset(&addr, 0, sizeof(addr));  // Zero out address structure
    addr.sin_family = AF_INET;       // Internet address

    hostent *host;  // Resolve name
    if ((host = gethostbyname(address.c_str())) == NULL) {
        std::cerr << "Failed to resolve name: " << address << std::endl;
        return; // Avoid crash
    }
    addr.sin_addr.s_addr = *((unsigned long *) host->h_addr_list[0]);

    addr.sin_port = htons(port);     // Assign port in network byte order
}

// Socket Code

Socket::Socket(int type, int protocol) {
    // Make a new socket
    if ((sockDesc = socket(PF_INET, type, protocol)) < 0) {
        std::cout<<"Socket creation failed (socket())"<<std::endl;
    }
}

Socket::Socket(int sockDesc) {
    this->sockDesc = sockDesc;
}

Socket::~Socket() {
    ::close(sockDesc);
    sockDesc = -1;
}

string Socket::getLocalAddress() {
    sockaddr_in addr;
    unsigned int addr_len = sizeof(addr);

    if (getsockname(sockDesc, (sockaddr *) &addr, (socklen_t *) &addr_len) < 0) {
        std::cout<<"Fetch of local address failed (getsockname())"<<std::endl;
    }
    return inet_ntoa(addr.sin_addr);
}

unsigned short Socket::getLocalPort() {
    sockaddr_in addr;
    unsigned int addr_len = sizeof(addr);

    if (getsockname(sockDesc, (sockaddr *) &addr, (socklen_t *) &addr_len) < 0) {
        std::cout<<"Fetch of local port failed (getsockname())"<<std::endl;
    }
    return ntohs(addr.sin_port);
}

void Socket::setLocalPort(unsigned short localPort) {
    // Bind the socket to its port
    sockaddr_in localAddr;
    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port = htons(localPort);

    if (bind(sockDesc, (sockaddr *) &localAddr, sizeof(sockaddr_in)) < 0) {
        std::cout<<"Set of local port failed (bind())"<<std::endl;
    }
}

void Socket::setLocalAddressAndPort(const string &localAddress,
                                    unsigned short localPort) {
    // Get the address of the requested host
    sockaddr_in localAddr;
    fillAddr(localAddress, localPort, localAddr);

    if (bind(sockDesc, (sockaddr *) &localAddr, sizeof(sockaddr_in)) < 0) {
        std::cout<<"Set of local address and port failed (bind())"<<std::endl;
    }
}

void Socket::shutdownSocket() {
    ::shutdown(sockDesc, SHUT_RDWR);
}

void Socket::cleanUp(){

}

unsigned short Socket::resolveService(const string &service,
                                      const string &protocol) {
    struct servent *serv;        /* Structure containing service information */

    if ((serv = getservbyname(service.c_str(), protocol.c_str())) == NULL)
        return atoi(service.c_str());  /* Service is port number */
    else
        return ntohs(serv->s_port);    /* Found port (network byte order) by name */
}

// CommunicatingSocket Code

CommunicatingSocket::CommunicatingSocket(int type, int protocol) : Socket(type, protocol) {
}

CommunicatingSocket::CommunicatingSocket(int newConnSD) : Socket(newConnSD) {
}

void CommunicatingSocket::connect(const string &foreignAddress,
                                  unsigned short foreignPort){
    // Get the address of the requested host
    sockaddr_in destAddr;
    fillAddr(foreignAddress, foreignPort, destAddr);

    // Try to connect to the given port
    if (::connect(sockDesc, (sockaddr *) &destAddr, sizeof(destAddr)) < 0) {
        std::cout<<"Connect failed (connect())"<<std::endl;
    }
}

void CommunicatingSocket::send(const void *buffer, int bufferLen) {
    if (::send(sockDesc, (raw_type *) buffer, bufferLen, 0) < 0) {
        std::cout<<"Send failed (send())"<<std::endl;
    }
}

int CommunicatingSocket::recv(void *buffer, int bufferLen) {
    int rtn;
    if ((rtn = ::recv(sockDesc, (raw_type *) buffer, bufferLen, 0)) < 0) {
        std::cout<<"Received failed (recv())"<<std::endl;
    }

    return rtn;
}

string CommunicatingSocket::getForeignAddress() {
    sockaddr_in addr;
    unsigned int addr_len = sizeof(addr);

    if (getpeername(sockDesc, (sockaddr *) &addr,(socklen_t *) &addr_len) < 0) {
        std::cout<<"Fetch of foreign address failed (getpeername())"<<std::endl;
    }
    return inet_ntoa(addr.sin_addr);
}

unsigned short CommunicatingSocket::getForeignPort(){
    sockaddr_in addr;
    unsigned int addr_len = sizeof(addr);

    if (getpeername(sockDesc, (sockaddr *) &addr, (socklen_t *) &addr_len) < 0) {
        std::cout<<"Fetch of foreign port failed (getpeername())"<<std::endl;
    }
    return ntohs(addr.sin_port);
}

// UDPSocket Code

UDPSocket::UDPSocket(): CommunicatingSocket(SOCK_DGRAM,IPPROTO_UDP) {
    setBroadcast();
}

UDPSocket::UDPSocket(unsigned short localPort) :
        CommunicatingSocket(SOCK_DGRAM, IPPROTO_UDP) {
    setLocalPort(localPort);
    setBroadcast();
}

UDPSocket::UDPSocket(const string &localAddress, unsigned short localPort) : CommunicatingSocket(SOCK_DGRAM, IPPROTO_UDP) {
    setLocalAddressAndPort(localAddress, localPort);
    setBroadcast();
}

void UDPSocket::setBroadcast() {
    // If this fails, we'll hear about it when we try to send.  This will allow
    // system that cannot broadcast to continue if they don't plan to broadcast
    int broadcastPermission = 1;
    setsockopt(sockDesc, SOL_SOCKET, SO_BROADCAST,
               (raw_type *) &broadcastPermission, sizeof(broadcastPermission));
}

void UDPSocket::disconnect(){
    sockaddr_in nullAddr;
    memset(&nullAddr, 0, sizeof(nullAddr));
    nullAddr.sin_family = AF_UNSPEC;

    // Try to disconnect
    if (::connect(sockDesc, (sockaddr *) &nullAddr, sizeof(nullAddr)) < 0) {

        if (errno != EAFNOSUPPORT) {
            std::cout<<"Disconnect failed (connect())"<<std::endl;
        }
    }
}

void UDPSocket::sendTo(const void *buffer, int bufferLen,
                       const string &foreignAddress, unsigned short foreignPort) {
    sockaddr_in destAddr;
    fillAddr(foreignAddress, foreignPort, destAddr);

    // Write out the whole buffer as a single message.
    if (sendto(sockDesc, (raw_type *) buffer, bufferLen, 0,
               (sockaddr *) &destAddr, sizeof(destAddr)) != bufferLen) {
        std::cout<<"Send failed (sendto())"<<std::endl;
    }
}

int UDPSocket::recvFrom(void *buffer, int bufferLen, string &sourceAddress,
                        unsigned short &sourcePort){
    sockaddr_in clntAddr;
    socklen_t addrLen = sizeof(clntAddr);
    int rtn;
    if ((rtn = recvfrom(sockDesc, (raw_type *) buffer, bufferLen, 0,
                        (sockaddr *) &clntAddr, (socklen_t *) &addrLen)) < 0) {
        std::cout<<"Receive failed (recvfrom())"<<std::endl;
    }
    sourceAddress = inet_ntoa(clntAddr.sin_addr);
    sourcePort = ntohs(clntAddr.sin_port);

    return rtn;
}
