#include "common_utilities/UDPSocket.hpp"
#include <chrono>

using namespace std;

const char* key = "The path of the righteous man is beset on all sides by the inequities of the "
        "selfish and the tyranny of evil men. Blessed is he who, in the name of "
        "charity and good will, shepherds the weak through the valley of the darkness. "
        "For he is truly his brother's keeper and the finder of lost children. And I "
        "will strike down upon thee with great vengeance and furious anger those who "
        "attempt to poison and destroy my brothers. And you will know I am the Lord "
        "when I lay my vengeance upon you\0";

uint8_t user[4] = {0xBF, 0x42, 0x76, 0xE9};

int main(int argc, char *argv[]){
    uint32_t client_IP, port = 8000;
    // create the IP broadcast Socket
    UDPSocketPtr receiver_socket = UDPSocketPtr(new UDPSocket(client_IP, port, false));

    if(argc == 2) {
        if(!receiver_socket->convertText2Byte(argv[1], &client_IP)) {
            printf(
                    "holy shit, I was not able to convert your client IP, are you sure it is an IP like 192.168.0.255 ??!!!\n");
            return -2;
        }
        receiver_socket.reset();
        receiver_socket = UDPSocketPtr(new UDPSocket(port, client_IP, false));
        printf("Starting rfid unlocker with client IP: %s:%d\n", argv[1],port);
    }else{
        printf("USAGE: rosrun common_utilities rfid_unlocker 192.168.0.104\n");
        printf("Please supply broadcast IP\n");
        return -1;
    }

    std::chrono::time_point<std::chrono::system_clock> t0, t1;

    t0 = std::chrono::system_clock::now();

    while(true){
        int numberOfBytes = receiver_socket->receiveUDPFromClient();
        if(numberOfBytes == strlen(key)) {
            uint j = 0;
            char output[strlen(key)];
            bool access_granted = true;
            for (int i=0; i<strlen(key); i++)
            {
                output[i] = receiver_socket->buf[i] ^ user[j];
                if(output[i]!=key[i])
                    access_granted = false;
                j++;
                if(j == sizeof(user))
                    j = 0;
            }
            if(access_granted) {
//                cout << "access granted" << endl;
                t0 = std::chrono::system_clock::now();
                FILE * pFile = fopen("/etc/sudoers.d/rfid","w");
                if (pFile!=NULL)
                {
                    fputs ("%sudo ALL=(ALL) NOPASSWD:ALL\n",pFile);
                    fclose (pFile);
                }
            }
        }

        t1 = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = t1-t0;
        if( diff.count() > 60.0 ){
//            cout << "access denied" << endl;
            FILE * pFile = fopen("/etc/sudoers.d/rfid","w");
            if (pFile!=NULL)
            {
                fputs ("%sudo ALL=(ALL:ALL) ALL\n",pFile);
                fclose (pFile);
            }
        }
        usleep(1000000);
    }
}
