#pragma once

// std
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string>
#include <ifaddrs.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <bitset>
#include <iostream>
#include <boost/shared_ptr.hpp>

#define MAXBUFLENGTH 1024

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#define BROADCAST_PORT 8000

using namespace std;

class UDPSocket{
public:
    /**
     * Creates a socket on the given server_IP and server_port and sets up the "connection" with the client.
     * Because it is UDP, there is no handshake, the socket just sends and listens to packages from the client_IP
     * and client_port
     * @param server_IP the server socket IP
     * @param server_port the server socket port
     * @param client_IP the client to send and receive UDP packets to/from
     * @param client_port the client port
     * @param exclusive receive exclusively packages from client
     */
    UDPSocket(const char* server_IP, int server_port, const char* client_IP, int client_port, bool exclusive=true);
    /**
     * Tries to guess the users IP and sets up the socket on Port 8000 and "connects" to client_IP on client_port
     * @param client_IP the client to send and receive UDP packets to/from
     * @param client_port the client port
     * @param exclusive receive exclusively packages from client
     */
    UDPSocket(const char* client_IP, int client_port, bool exclusive=true);


    UDPSocket(const char* client_IP, int client_port, int server_port, bool exclusive=true);
    /**
     * Creates a broadcast socket on port
     * @param port on this port
     * @param broadcastIP use this broadcast IP
     * @param broadcaster if true this binds the port to the broadcast port
     */
    UDPSocket(int port, int broadcastIP=0xffffffff, bool broadcaster=false);
    ~UDPSocket();

    /**
     * Receive ROS master IP
     * @return IP
     */
    uint32_t receiveHostIP(char *hostname);

    /**
      * Broadcast ROS master IP
      * @param IP broadcast this IP
      * @return successfully broadcasted
      */
    bool broadcastHostIP(uint32_t IP);
    /**
      * Broadcast ROS master IP of the machine this node was run on
      * @param hostname 
      * @return successfully broadcasted
      */
    bool broadcastHostIP(char* hostname);

    /**
     * Listens for UDP package containing lighthouse tracking data
     * @param sensorID sensor id
     * @param lighthouse witch lighthouse
     * @param axis
     * @param sweepDuration
     * @return
     */
    bool receiveSensorData(uint32_t &sensorID, bool &lighthouse, bool &axis, uint16_t &sweepDuration);

    /**
     * Converts a byte internet address to a human readable address
     * @param inet
     * @param inet_str
     * @return success
     */
    bool convertByte2Text(uint32_t inet, char *inet_str);
    /**
     * Converts a human readable address to abyte internet address
     * @param inet_str
     * @param inet
     * @return success
     */
    bool convertText2Byte(char *inet_str, uint32_t *inet);

    pair<uint32_t,string> myIP;

    /**
    * receive from anyone
    * @return received number of bytes
    */
    int receiveUDP();
    /**
    * receive from client
    * @return received number of bytes
    */
    int receiveUDPFromClient();
    char buf[MAXBUFLENGTH];
private:
    /**
     * Sets the UDP package receive and send timeout
     * @param usecs time in microseconds
     * @return success
     */
    bool setTimeOut(int usecs);

    /**
     * Tries to guess your IP
     * @param ip your IP
     * @param success (fails if I can't find a valid IP for wifi or ethernet adapter)
     */
    bool whatsMyIP(string &IP);

    bool initialized = false;
    /**
     * send to client
     * @return success
     */
    bool sendUDPToClient();
    /**
     * broadcast
     * @return success
     */
    bool broadcastUDP();
private:
    int sockfd; //* socket
    struct sockaddr_in server_addr; /* server's addr */
    struct sockaddr_in broadcast_addr; /* server's addr */
    struct sockaddr_in client_addr; /* client addr */
    socklen_t client_addr_len, server_addr_len, broadcast_addr_len; /* byte size of addresses */
    ssize_t numbytes; /* message byte size */
    struct addrinfo *servinfo;
    bool exclusive;
};

typedef boost::shared_ptr<UDPSocket> UDPSocketPtr;
