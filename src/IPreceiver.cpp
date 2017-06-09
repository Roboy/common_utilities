#include "common_utilities/UDPSocket.hpp"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "IPreceiver" );

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    uint32_t client_IP, host_IP, broadcastIP;

    // create the IP broadcast Socket
    UDPSocketPtr receiver_socket = UDPSocketPtr(new UDPSocket(client_IP, BROADCAST_PORT, false));

    switch(argc){
        case 1:
            host_IP = receiver_socket->myIP.first;
            ROS_INFO("Starting IP broadcaster with Host IP: %s", receiver_socket->myIP.second.c_str());
            break;
        case 3:
            if(!receiver_socket->convertText2Byte(argv[1], &client_IP)) {
                ROS_ERROR(
                        "holy shit, I was not able to convert your IP, are you sure it is an IP like 192.168.0.100 ??!!!");
                return -2;
            }
            if(!receiver_socket->convertText2Byte(argv[2], &broadcastIP)) {
                ROS_ERROR(
                        "holy shit, I was not able to convert your broadcast IP, are you sure it is an IP like 192.168.0.255 ??!!!");
                return -2;
            }
            receiver_socket.reset();
            receiver_socket = UDPSocketPtr(new UDPSocket(BROADCAST_PORT, broadcastIP, false));
            ROS_INFO("Starting IP receiver with Host IP: %s:%d and broadcastIP: %s", argv[1],BROADCAST_PORT, argv[2]);
            break;
        default:
            ROS_ERROR("USAGE: rosrun common_utilities IPreceiver [192.168.0.100] [192.168.0.255]");
            ROS_INFO("The IP and broadcast IP is optional. If you dont supply it, I will try to guess your IP and "
                             "use 255.255.255.255 as the broadcast IP");
            return -1;
    }

    ros::Rate rate(1);
    while(ros::ok()){
        ROS_INFO_THROTTLE(10,"listening for HOST IP");
        host_IP = receiver_socket->receiveHostIP();
        if(host_IP!=0){
            char IP[4];
            receiver_socket->convertByte2Text(host_IP,IP);
            ROS_INFO("Received HOST IP: %s",IP);
        }
        rate.sleep();
    }
}