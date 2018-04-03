#include <ros/ros.h>
#include "common_utilities/UDPSocket.hpp"

const char* key = "The path of the righteous man is beset on all sides by the inequities of the "
        "selfish and the tyranny of evil men. Blessed is he who, in the name of "
        "charity and good will, shepherds the weak through the valley of the darkness. "
        "For he is truly his brother's keeper and the finder of lost children. And I "
        "will strike down upon thee with great vengeance and furious anger those who "
        "attempt to poison and destroy my brothers. And you will know I am the Lord "
        "when I lay my vengeance upon you\0";

int main(int argc, char *argv[]){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "ROS_MASTER_URI_broadcaster");
    }

    char hostname[20];
    /* Open the command for reading. */
    FILE *fp = popen("hostname", "r");
    fgets(hostname, sizeof(hostname)-1, fp);

    // create the IP broadcast Socket
    UDPSocketPtr broadcast_socket = UDPSocketPtr(new UDPSocket(BROADCAST_PORT, true));

    // encrypt key
    uint8_t user[4] = {0xBF, 0x42, 0x76, 0xE9};
    char output[strlen(key)];
    int j = 0;
    for (int i=0; i<strlen(key); i++)
    {
        output[i] = key[i] ^ user[j];
        j++;
        if(j == sizeof(user))
            j = 0;
    }
//    printf("%s\n%s\n", key, output);

    while(ros::ok()){
        ROS_INFO("broadcasting HOST IP");
        broadcast_socket->broadcastHostIP(output,strlen(key));
        usleep(5000000);
    }
}
