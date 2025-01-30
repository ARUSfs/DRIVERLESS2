#include <cstring>
#include <string>
#include <thread>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <chrono>
#include <fcntl.h>
#include <cerrno>
#include <csignal>

// Variables globales
std::atomic<bool> HV_ON{false};
std::atomic<bool> running{true}; // Variable para controlar la ejecución del programa
int socket_can_rx = -1;
int socket_can_tx = -1;
uint8_t mission;
std::string baseCommand = "ros2 launch common_meta ";

// Sincronización
std::mutex time_mtx;

// Último tiempo de recepción de HV_ON
std::chrono::steady_clock::time_point last_HV_ON_time;

// Configuración
const std::string CAN_INTERFACE_RX = "can0"; // Interfaz CAN para recibir mensajes
const std::string CAN_INTERFACE_TX = "can1"; // Interfaz CAN para enviar Heartbeat

// Timeout como duración de 2 segundos
const std::chrono::seconds HV_ON_TIMEOUT(2);

// Función para inicializar un socket SocketCAN
bool initCanSocket(int &sock, const std::string &ifname)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    // Crear el socket CAN
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0)
    {
        perror(("Error al crear el socket CAN para " + ifname).c_str());
        return false;
    }

    // Configurar el socket en modo no bloqueante
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags == -1)
    {
        perror("fcntl F_GETFL");
        close(sock);
        return false;
    }

    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        perror("fcntl F_SETFL");
        close(sock);
        return false;
    }

    // Obtener el índice de la interfaz CAN
    std::strcpy(ifr.ifr_name, ifname.c_str());
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        perror(("Error al obtener el índice de la interfaz CAN " + ifname).c_str());
        close(sock);
        return false;
    }

    // Vincular el socket a la interfaz CAN
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror(("Error al vincular el socket CAN " + ifname).c_str());
        close(sock);
        return false;
    }

    printf("SocketCAN inicializado correctamente en %s\n", ifname.c_str());
    return true;
}

// Función para cerrar ambos sockets CAN
void closeCanSockets()
{
    if (socket_can_rx != -1)
    {
        close(socket_can_rx);
        socket_can_rx = -1;
        printf("SocketCAN_RX (%s) cerrado\n", CAN_INTERFACE_RX.c_str());
    }
    if (socket_can_tx != -1)
    {
        close(socket_can_tx);
        socket_can_tx = -1;
        printf("SocketCAN_TX (%s) cerrado\n", CAN_INTERFACE_TX.c_str());
    }
}

// Función para lanzar la misión de forma asíncrona
std::thread mission_thread;

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

        printf("Lanzando misión: %s\n", command.c_str());
        int ret = system(command.c_str());

        if (ret == 0)
        {
            printf("Misión lanzada exitosamente\n");
        }
        else
        {
            printf("Fallo al lanzar la misión\n");
        }
    });
}

// Función para enviar Heartbeat
void pubHeartBeat()
{
    struct can_frame frame;
    frame.can_id = 0x183;
    frame.can_dlc = 1;
    frame.data[0] = 0x00;

    int nbytes = write(socket_can_tx, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame))
    {
        perror("Error al enviar Heartbeat");
    }
}

// Función para monitorear HV_ON
// void monitorHV(std::chrono::steady_clock::time_point program_start_time)
// {
//     while (running)
//     {
//         // std::this_thread::sleep_for(std::chrono::milliseconds(100));

//         // std::chrono::steady_clock::time_point last_time_local;

//         {
//             std::lock_guard<std::mutex> lock(time_mtx);
//             last_time_local = last_HV_ON_time;
//         }

//         auto now = std::chrono::steady_clock::now();

//         if (last_time_local == std::chrono::steady_clock::time_point::min())
//         {
//             // No se ha recibido HV_ON aún
//             auto duration_since_start = now - program_start_time;
//             if (duration_since_start > HV_ON_TIMEOUT)
//             {
//                 printf("No se ha recibido HV_ON en los primeros %d segundos. Finalizando el programa.\n", (int)HV_ON_TIMEOUT.count());
//                 HV_ON = false;
//                 running = false;
//                 closeCanSockets();
//                 break;
//             }
//         }
//         else
//         {
//             auto duration_since_last = now - last_time_local;

//             if (duration_since_last > HV_ON_TIMEOUT)
//             {
//                 printf("No se ha recibido HV_ON en los últimos %d segundos. Finalizando el programa.\n", (int)HV_ON_TIMEOUT.count());
//                 HV_ON = false;
//                 running = false;

//                 // Si hay una misión en ejecución, terminarla
//                 if (mission_thread.joinable())
//                 {
//                     printf("Terminando misión en ejecución...\n");
//                     mission_thread.detach();
//                 }

//                 closeCanSockets();
//                 break;
//             }
//         }
//     }
// }

int main(int argc, char **argv)
{
    // auto program_start_time = std::chrono::steady_clock::now();

    // Inicializar last_HV_ON_time a un valor que indique que no se ha recibido HV_ON
    // last_HV_ON_time = std::chrono::steady_clock::time_point::min();

    // Manejar señales para asegurarse de que los sockets se cierren adecuadamente
    signal(SIGINT, [](int signum) {
        printf("Interrupción recibida. Cerrando sockets...\n");
        running = false;
        closeCanSockets();
        exit(signum);
    });

    // Inicializar SocketCAN para recepción (can0)
    if (!initCanSocket(socket_can_rx, CAN_INTERFACE_RX))
    {
        fprintf(stderr, "Fallo al inicializar SocketCAN_RX (%s)\n", CAN_INTERFACE_RX.c_str());
        return EXIT_FAILURE;
    }

    // Inicializar SocketCAN para transmisión (can1)
    if (!initCanSocket(socket_can_tx, CAN_INTERFACE_TX))
    {
        fprintf(stderr, "Fallo al inicializar SocketCAN_TX (%s)\n", CAN_INTERFACE_TX.c_str());
        closeCanSockets();
        return EXIT_FAILURE;
    }

    // Iniciar el hilo para monitorear HV_ON
    // std::thread hv_monitor_thread(monitorHV, program_start_time);

    // Bucle principal
    while (running)
    {
        pubHeartBeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        

        struct can_frame frame;
        int nbytes = read(socket_can_rx, &frame, sizeof(struct can_frame));

        if (nbytes > 0)
        {
            // Procesar el frame recibido
            uint32_t id = frame.can_id;
            uint8_t data0 = frame.data[0];
            uint8_t data1 = frame.data[1];

            // Imprimir el frame recibido
            printf("Frame recibido: ID=0x%X, data0=0x%02X, data1=0x%02X\n", id, data0, data1);

            if (id == 0x186)
            {
                if (data0 == 0x00 && data1 == 0x0F)
                {
                    HV_ON = true;
                    {
                        std::lock_guard<std::mutex> lock(time_mtx);
                        last_HV_ON_time = std::chrono::steady_clock::now();
                    }
                    printf("HV ON\n");
                }
                else
                {
                    // Si recibes un mensaje con ID 0x186 que no es HV_ON, puedes considerar que HV_ON está apagado
                    HV_ON = false;
                    printf("HV OFF\n");
                }
            }
            else if (id == 0x185 && data0 == 0x01)
            {
                if (HV_ON)
                {
                    printf("Misión recibida\n");
                    mission = data1;
                    launchMission();
                    // No cerrar sockets ni salir del bucle
                }
                else
                {
                    printf("Misión recibida pero HV_ON no está activo. Ignorando misión.\n");
                }
            }
        }
        else if (nbytes < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                perror("Error al leer del socket CAN_RX");
            }
            // Si no hay datos, continuar sin hacer nada
        }

        // Dormir un poco para evitar consumir CPU en exceso
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Finalizar el hilo de monitoreo
    // if (hv_monitor_thread.joinable())
    // {
    //     hv_monitor_thread.join();
    // }

    // Esperar a que el hilo de la misión termine
    if (mission_thread.joinable())
    {
        mission_thread.join();
    }

    closeCanSockets();
    return EXIT_SUCCESS;
}
