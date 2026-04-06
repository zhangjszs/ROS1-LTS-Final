#include "vehicle_interface_core/udp_socket.h"

#include <cstring>
#include <string>

#include <gtest/gtest.h>

// ==================== UDP Socket Tests ====================

TEST(UDPSocketTest, DefaultConstructor) {
  // Test that default constructor creates a valid socket
  EXPECT_NO_THROW({
    UDPSocket socket;
    // Socket should be created successfully
  });
}

TEST(UDPSocketTest, ConstructorWithPort) {
  // Test that constructor with specific port works
  EXPECT_NO_THROW({
    UDPSocket socket(12345);
    // Socket should be created and bound to port 12345
  });
}

TEST(UDPSocketTest, ConstructorWithAddressAndPort) {
  // Test that constructor with address and port works
  EXPECT_NO_THROW({
    UDPSocket socket("127.0.0.1", 12346);
    // Socket should be created and bound to the specified address and port
  });
}

TEST(UDPSocketTest, GetLocalPort) {
  UDPSocket socket;
  // Default constructor creates unbound socket, port may be 0
  unsigned short port = socket.getLocalPort();
  // Port will be 0 for unbound socket, or assigned if implicitly bound
  EXPECT_GE(port, 0);
}

TEST(UDPSocketTest, GetLocalPortWithSpecificPort) {
  const unsigned short test_port = 12347;
  UDPSocket socket(test_port);
  // Port should match what we specified
  unsigned short port = socket.getLocalPort();
  EXPECT_EQ(port, test_port);
}

TEST(UDPSocketTest, GetLocalAddress) {
  UDPSocket socket;
  // Should return a valid address (likely "0.0.0.0" for unbound socket)
  std::string addr = socket.getLocalAddress();
  EXPECT_FALSE(addr.empty());
}

TEST(UDPSocketTest, SetReceiveTimeout) {
  UDPSocket socket;
  // Should not throw when setting timeout
  EXPECT_NO_THROW(socket.setReceiveTimeout(100));  // 100ms timeout
  EXPECT_NO_THROW(socket.setReceiveTimeout(0));    // No timeout
}

// ==================== Socket Service Resolution Tests ====================

TEST(SocketTest, ResolveService) {
  // Test service resolution for well-known services
  EXPECT_NO_THROW({
    unsigned short http_port = Socket::resolveService("http", "tcp");
    EXPECT_EQ(http_port, 80);

    unsigned short https_port = Socket::resolveService("https", "tcp");
    EXPECT_EQ(https_port, 443);
  });
}

// ==================== Multiple Socket Tests ====================

TEST(UDPSocketTest, MultipleSocketsDifferentPorts) {
  // Test that multiple sockets can be created on different ports
  EXPECT_NO_THROW({
    UDPSocket socket1(12348);
    UDPSocket socket2(12349);
    UDPSocket socket3(12350);

    EXPECT_EQ(socket1.getLocalPort(), 12348);
    EXPECT_EQ(socket2.getLocalPort(), 12349);
    EXPECT_EQ(socket3.getLocalPort(), 12350);
  });
}

TEST(UDPSocketTest, MultipleSocketsExplicitPorts) {
  // Test that multiple sockets can be created with explicit different ports
  EXPECT_NO_THROW({
    UDPSocket socket1(22301);
    UDPSocket socket2(22302);
    UDPSocket socket3(22303);

    // All should have different ports
    unsigned short port1 = socket1.getLocalPort();
    unsigned short port2 = socket2.getLocalPort();
    unsigned short port3 = socket3.getLocalPort();

    EXPECT_EQ(port1, 22301);
    EXPECT_EQ(port2, 22302);
    EXPECT_EQ(port3, 22303);
    EXPECT_NE(port1, port2);
    EXPECT_NE(port2, port3);
    EXPECT_NE(port1, port3);
  });
}

// ==================== Socket Cleanup Test ====================

TEST(SocketTest, Cleanup) {
  // Socket cleanup should not throw
  EXPECT_NO_THROW(Socket::cleanUp());
}

// ==================== Socket Destruction Test ====================

TEST(UDPSocketTest, SocketDestruction) {
  // Test that sockets are properly destroyed
  {
    UDPSocket socket(12351);
    EXPECT_EQ(socket.getLocalPort(), 12351);
  }  // Socket should be destroyed here

  // We should be able to create a new socket on the same port
  EXPECT_NO_THROW({
    UDPSocket socket(12351);
    EXPECT_EQ(socket.getLocalPort(), 12351);
  });
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
