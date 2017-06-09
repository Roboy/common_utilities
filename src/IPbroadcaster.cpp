#include "common_utilities/UDPSocket.hpp"

int main(int argc, char *argv[]){
    // create the IP broadcast Socket
    UDPSocketPtr broadcast_socket = UDPSocketPtr(new UDPSocket(BROADCAST_PORT));

    uint32_t host_IP, broadcastIP;

    switch(argc){
        case 1:
            host_IP = broadcast_socket->myIP.first;
            ROS_INFO("Starting IP broadcaster with Host IP: %s", broadcast_socket->myIP.second.c_str());
            break;
        case 3:
            ROS_INFO("IP: %s \t\t broadcastIP: %s", argv[1], argv[2]);
            if(!broadcast_socket->convertText2Byte(argv[1], &host_IP)) {
                ROS_ERROR(
                        "holy shit, I was not able to convert your IP, are you sure it is an IP like 192.168.0.100 ??!!!");
                return -2;
            }
            if(!broadcast_socket->convertText2Byte(argv[2], &broadcastIP)) {
                ROS_ERROR(
                        "holy shit, I was not able to convert your broadcast IP, are you sure it is an IP like 192.168.0.255 ??!!!");
                return -2;
            }
            broadcast_socket.reset();
            broadcast_socket = UDPSocketPtr(new UDPSocket(BROADCAST_PORT, broadcastIP));
            ROS_INFO("Starting IP broadcaster with Host IP: %s", argv[1]);
            break;
        default:
            ROS_ERROR("USAGE: rosrun common_utilities IPbroadcaster [192.168.0.100] [192.168.0.255]");
            ROS_INFO("The IP and broadcast IP is optional. If you dont supply it, I will try to guess your IP and "
                             "use 255.255.255.255 as the broadcast IP");
            return -1;
    }

    while(true){
        ROS_INFO("broadcasting HOST IP");
        broadcast_socket->broadcastHostIP();
        usleep(5000000);
    }
}
