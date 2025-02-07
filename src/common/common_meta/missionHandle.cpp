#include <cstring>
#include <string>
#include <thread>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <condition_variable>
#include <iostream>
#include <chrono>
#include <fcntl.h>
#include <cerrno>
#include <csignal>


bool HV_ON = false;
bool running = true; // Variable para controlar la ejecución del programa
int socket_can_rx = -1;
int socket_can_tx = -1;
uint8_t mission;
std::string baseCommand = "bash -c \'ros2 launch common_meta ";


const std::string CAN_INTERFACE_RX = "can0"; // Interfaz CAN para recibir mensajes
const std::string CAN_INTERFACE_TX = "can1"; // Interfaz CAN para enviar Heartbeat

std::thread mission_thread;
std::thread heartbeat_thread;



void launchMission()
{
    mission_thread = std::thread([=]() {
        std::string command = baseCommand;

        switch (mission)
        {
            case 1:
                command += "acceleration_launch.py";
                break;
            case 2:
                command += "skidpad_launch.py";
                break;
            case 3:
                command += "trackdrive_launch.py";
                break;
            case 4:
                command += "inspection_launch.py";
                break;
            case 5:
                command += "inspection_launch.py";
                break;
            case 6:
                command += "autocross_launch.py";
                break;
            default:
                printf("Misión desconocida: %d\n", mission);
                return;
        }

        command += "\'";

        printf("Launching mission: %s\n", command.c_str());
        int ret = system(command.c_str());
        if (ret == 0)
        {
            std::cout << "Mission launched succesfully" << std::endl;
        }
        else
        {
            std::cout << "Error launching mission" << std::endl;
        }
    });
}


void mission_loop()
{
    while (running)
    {
        struct can_frame frame;
        int nbytes = read(socket_can_rx, &frame, sizeof(struct can_frame));

        if (nbytes > 0)
        {
            // Procesar el frame recibido
            uint32_t id = frame.can_id;
            uint8_t data0 = frame.data[0];
            uint8_t data1 = frame.data[1];

            if (id == 0x186)
            {
                if (data0 == 0x00 && data1 == 0x0F)
                {
                    HV_ON = true;
                    std::cout << "HV ON" << std::endl;
                }
                else
                {
                    // Si recibes un mensaje con ID 0x186 que no es HV_ON, puedes considerar que HV_ON está apagado
                    HV_ON = false;
                    std::cout << "HV OFF" << std::endl;
                }
            }
            else if (id == 0x185 && data0 == 0x01)
            {
                if (HV_ON)
                {
                    std::cout << "Mission received" << std::endl;
                    mission = data1;
                    launchMission();
                    // No cerrar sockets ni salir del bucle
                }
                else
                {
                    std::cout << "Mission received but HV OFF. Ignoring mission." << std::endl;
                }
            }
        }
        else if (nbytes < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                perror("Error reading socket CAN_RX");
            }
        }
    }
}

// Función para enviar Heartbeat
void pubHeartBeat()
{
    struct can_frame frame;
    frame.can_id = 0x140;
    frame.can_dlc = 1;
    frame.data[0] = 0x00;

    int nbytes = write(socket_can_tx, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame))
    {
        perror("Error al enviar Heartbeat");
    }
}

void heartbeat_loop()
{
    while (running)
    {
        pubHeartBeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


bool initCanSocket(int &sock, const std::string &ifname)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    // Create CAN socket
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror(("Error while opening CAN socket " + ifname).c_str());
        return false;
    }

    // Set socket to non-blocking
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags == -1) {
        perror("fcntl F_GETFL");
        close(sock);
        return false;
    }

    // Obtain the interface index of the CAN interface
    std::strcpy(ifr.ifr_name, ifname.c_str());
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror(("Error getting interface index " + ifname).c_str());
        close(sock);
        return false;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror(("Error linking socket CAN " + ifname).c_str());
        close(sock);
        return false;
    }

    printf("SocketCAN initialized for %s\n", ifname.c_str());
    return true;
}

void closeCanSockets()
{
    if (socket_can_rx != -1)
    {
        close(socket_can_rx);
        socket_can_rx = -1;
        printf("SocketCAN_RX (%s) closed\n", CAN_INTERFACE_RX.c_str());
    }
    if (socket_can_tx != -1)
    {
        close(socket_can_tx);
        socket_can_tx = -1;
        printf("SocketCAN_TX (%s) closed\n", CAN_INTERFACE_TX.c_str());
    }
}



int main(int argc, char **argv)
{
    signal(SIGINT, [](int signum) {
        printf("Interruption received. Closing sockets...\n");
        running = false;
        closeCanSockets();
        exit(signum);
    });

    if (!initCanSocket(socket_can_rx, CAN_INTERFACE_RX))
    {
        fprintf(stderr, "Failure initializing SocketCAN_RX (%s)\n", CAN_INTERFACE_RX.c_str());
        return EXIT_FAILURE;
    }

    if (!initCanSocket(socket_can_tx, CAN_INTERFACE_TX))
    {
        fprintf(stderr, "Failure initializing SocketCAN_TX (%s)\n", CAN_INTERFACE_TX.c_str());
        closeCanSockets();
        return EXIT_FAILURE;
    }

    std::thread mission_thread(mission_loop);
    std::thread heartbeat_thread(heartbeat_loop);

    
    mission_thread.detach();
    heartbeat_thread.join();

    return EXIT_SUCCESS;
}
