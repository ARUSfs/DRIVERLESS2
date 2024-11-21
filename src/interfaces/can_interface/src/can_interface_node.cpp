/**
 * @file can_interface_node.cpp
 * @brief CanInterface node implementaion for ARUS Team Driverless pipeline
 */

#include "can_interface/can_interface_node.hpp"

/**
 * @class CanInterface
 * @brief CanInterface class 
 * 
 * This class initialize the canlib library, create a thread for each CAN line and create the handles.
 */ 
CanInterface::CanInterface() : Node("can_interface")
{
    canInitializeLibrary(); 
    std::cout << "CANlib library initialised" << std::endl;
    canStatus stat;
    int chanCount;
    canGetNumberOfChannels(&chanCount);
    printf("%d channels.\n", chanCount);

    std::thread thread_0(&CanInterface::read_can0, this);
    std::thread thread_1(&CanInterface::read_can1, this);

    thread_0.join();
    thread_1.join();
}

/**
 * @brief Fuction to import the CSV as a key and a vector, row by row.
 * 
 */
void CanInterface::read_csv()
{
    std::ifstream file("can_interface/data.csv");
    std::string line;
    std::map<std::string, std::vector<std::string>> dataMap;

    // Skip the first line (headers)
    if (std::getline(file, line)) {
        // The header line is read but not used
    }

    // Read the rest of the lines and process them
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string id;
        std::string value;
        std::vector<std::string> values;

        // Read the first column as the key (id)
        if (std::getline(ss, id, ',')) {
            // Read the rest of the columns and store them in the vector
            while (std::getline(ss, value, ',')) {
                values.push_back(value);
            }
        }

        // Insert the key and associated vector into the map
        dataMap[id] = values;
    }

    // Close the file
    file.close();
}


/**
 * @brief Function to know the status of the CAN
 * 
 * @return the error in case there is a problem with the driver you are connected to
 */
void CanInterface::check_can(canStatus stat)
{
    if(stat != canOK){
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        printf("failed, stat=%d (%s) \n", (int)stat, buf);
    }
}

/**
 * @brief Function which read the CAN line 0
 * 
 */
void CanInterface::read_can0()
{   
    canStatus stat;

    CanHandle hndR0 = canOpenChannel(0, canOPEN_ACCEPT_VIRTUAL);
    if (hndR0 < 0){
        printf("canOpenChannel() failed, %d \n", hndR0);
        return;
    }else{
        printf("can0 enabled for reading \n");
    }

    stat = canSetBusParams(hndR0, canBITRATE_1M,0,0,0,0,0);
    if (stat!=canOK){
        printf("canSetBusParams failed, status=%d\n", stat);
        exit(1);
    }
    
    stat = canBusOn(hndR0);
    CanInterface::check_can(stat);

    long id;
    uint8_t msg[8];
    unsigned int dlc;
    unsigned int flag;
    unsigned long time;
    stat = canReadWait(hndR0, &id, &msg, &dlc, &flag, &time, 300);
    int8_t subId = msg[0];

    if(stat == canOK){
        std::cout << "CAN0:" << id << subId << msg[8];
    }

    else if(stat == canERR_NOMSG){
        std::cout << "No messages on CAN0";
    }

    stat = canBusOff(hndR0);
    stat = canClose(hndR0);
}

/**
 * @brief Function which read the CAN line 1
 * 
 */
void CanInterface::read_can1()
{   
    canStatus stat;
    CanHandle hndR1 = canOpenChannel(1, canOPEN_ACCEPT_VIRTUAL);
    if (hndR1 < 0){
        printf("canOpenChannel() failed, %d\n", hndR1);
        return;
    }else{
        printf("can1 enabled for reading \n");
    }

    stat = canBusOn(hndR1);

    long id;
    uint8_t msg[8];
    unsigned int dlc;
    unsigned int flag;
    unsigned long time;
    stat = canReadWait(hndR1, &id, &msg, &dlc, &flag, &time, 300);
    uint8_t subId = msg[0];


    if(stat == canOK){
        std::cout << "CAN1:" << id << subId << msg[8];
    }

    else if(stat == canERR_NOMSG){
        std::cout << "No messages on CAN1";
    }    

    stat = canBusOff(hndR1);
    stat = canClose(hndR1);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanInterface>());
    rclcpp::shutdown();
    return 0;
}
