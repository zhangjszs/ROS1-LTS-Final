//
// Created by wss on 9/9/17.
//

#ifndef VEHICLE_INTERFACE_CORE_UDP_SOCKET_H
#define VEHICLE_INTERFACE_CORE_UDP_SOCKET_H

#include <string>            // For string
#include <exception>         // For exception class
#include <cstring>

using namespace std;

/**
 *   Base class representing basic communication endpoint
 */
class Socket {
public:
    /**
     *   Close and deallocate this socket
     */
    ~Socket();

    /**
     *   Get the local address
     *   @return local address of socket
     *   @exception SocketException thrown if fetch fails
     */
    string getLocalAddress();

    /**
     *   Get the local port
     *   @return local port of socket
     *   @exception SocketException thrown if fetch fails
     */
    unsigned short getLocalPort();

    /**
     *   Set the local port to the specified port and the local address
     *   to any interface
     *   @param localPort local port
     *   @exception SocketException thrown if setting local port fails
     */
    void setLocalPort(unsigned short localPort);

    /**
     *   Set the local port to the specified port and the local address
     *   to the specified address.  If you omit the port, a random port
     *   will be selected.
     *   @param localAddress local address
     *   @param localPort local port
     *   @exception SocketException thrown if setting local port or address fails
     */
    void setLocalAddressAndPort(const string &localAddress,
                                unsigned short localPort = 0);

    /**
     *   If WinSock, unload the WinSock DLLs; otherwise do nothing.  We ignore
     *   this in our sample client code but include it in the library for
     *   completeness.  If you are running on Windows and you are concerned
     *   about DLL resource consumption, call this after you are done with all
     *   Socket instances.  If you execute this on Windows while some instance of
     *   Socket exists, you are toast.  For portability of client code, this is
     *   an empty function on non-Windows platforms so you can always include it.
     *   @param buffer buffer to receive the data
     *   @param bufferLen maximum number of bytes to read into buffer
     *   @return number of bytes read, 0 for EOF, and -1 for error
     *   @exception SocketException thrown WinSock clean up fails
     */
    static void cleanUp();

    /**
     *   Resolve the specified service for the specified protocol to the
     *   corresponding port number in host byte order
     *   @param service service to resolve (e.g., "http")
     *   @param protocol protocol of service to resolve.  Default is "tcp".
     */
    static unsigned short resolveService(const string &service,
                                         const string &protocol = "tcp");

    /**
     *   Shutdown the socket to interrupt blocking calls
     */
    void shutdownSocket();

private:
    // Prevent the user from trying to use value semantics on this object
    Socket(const Socket &sock);
    void operator=(const Socket &sock);

protected:
    int sockDesc;              // Socket descriptor
    Socket(int type, int protocol);
    Socket(int sockDesc);
};

/**
 *   Socket which is able to connect, send, and receive
 */
class CommunicatingSocket : public Socket {
public:
    /**
     *   Establish a socket connection with the given foreign
     *   address and port
     *   @param foreignAddress foreign address (IP address or name)
     *   @param foreignPort foreign port
     *   @exception SocketException thrown if unable to establish connection
     */
    void connect(const string &foreignAddress, unsigned short foreignPort);

    /**
     *   Write the given buffer to this socket.  Call connect() before
     *   calling send()
     *   @param buffer buffer to be written
     *   @param bufferLen number of bytes from buffer to be written
     *   @exception SocketException thrown if unable to send data
     */
    void send(const void *buffer, int bufferLen);

    /**
     *   Read into the given buffer up to bufferLen bytes data from this
     *   socket.  Call connect() before calling recv()
     *   @param buffer buffer to receive the data
     *   @param bufferLen maximum number of bytes to read into buffer
     *   @return number of bytes read, 0 for EOF, and -1 for error
     *   @exception SocketException thrown if unable to receive data
     */
    int recv(void *buffer, int bufferLen);

    /**
     *   Get the foreign address.  Call connect() before calling recv()
     *   @return foreign address
     *   @exception SocketException thrown if unable to fetch foreign address
     */
    string getForeignAddress();

    /**
     *   Get the foreign port.  Call connect() before calling recv()
     *   @return foreign port
     *   @exception SocketException thrown if unable to fetch foreign port
     */
    unsigned short getForeignPort();

protected:
    CommunicatingSocket(int type, int protocol);
    CommunicatingSocket(int newConnSD);
};


/**
  *   UDP socket class
  */
class UDPSocket : public CommunicatingSocket {
public:
    /**
     *   Construct a UDP socket
     *   @exception SocketException thrown if unable to create UDP socket
     */
    UDPSocket();

    /**
     *   Construct a UDP socket with the given local port
     *   @param localPort local port
     *   @exception SocketException thrown if unable to create UDP socket
     */
    UDPSocket(unsigned short localPort);

    /**
     *   Construct a UDP socket with the given local port and address
     *   @param localAddress local address
     *   @param localPort local port
     *   @exception SocketException thrown if unable to create UDP socket
     */
    UDPSocket(const string &localAddress, unsigned short localPort);

    /**
     *   Unset foreign address and port
     *   @return true if disassociation is successful
     *   @exception SocketException thrown if unable to disconnect UDP socket
     */
    void disconnect();

    /**
     *   Send the given buffer as a UDP datagram to the
     *   specified address/port
     *   @param buffer buffer to be written
     *   @param bufferLen number of bytes to write
     *   @param foreignAddress address (IP address or name) to send to
     *   @param foreignPort port number to send to
     *   @return true if send is successful
     *   @exception SocketException thrown if unable to send datagram
     */
    void sendTo(const void *buffer, int bufferLen, const string &foreignAddress,
                unsigned short foreignPort);

    /**
     *   Read read up to bufferLen bytes data from this socket.  The given buffer
     *   is where the data will be placed
     *   @param buffer buffer to receive data
     *   @param bufferLen maximum number of bytes to receive
     *   @param sourceAddress address of datagram source
     *   @param sourcePort port of data source
     *   @return number of bytes received and -1 for error
     *   @exception SocketException thrown if unable to receive datagram
     */
    int recvFrom(void *buffer, int bufferLen, string &sourceAddress,
                 unsigned short &sourcePort);

private:
    void setBroadcast();
};

#endif // VEHICLE_INTERFACE_CORE_UDP_SOCKET_H
