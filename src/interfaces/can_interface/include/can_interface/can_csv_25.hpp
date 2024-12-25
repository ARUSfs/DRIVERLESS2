/**
 * @file can_csv_25.hpp
 * @author Álvaro Galisteo Bermúdez (galisbermo03@gmail.com)
 * @brief Header file for the CAN interface node. 
 * Contains the class definition and the declaration of the methods 
 * used in the parse.
 * @version 0.1
 * @date 25-12-2024
 */
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <thread>
#include <thread>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <fcntl.h>
#include <common_msgs/msg/cmd.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdexcept>

using namespace std;

/**
 * @brief Data structure for processing can.csv data in the parse function
 * 
 */
struct CANParseConfig {
    uint8_t startByte;       // Start byte position
    uint8_t endByte;         // End byte position
    float scale;             // Scaling factor
    float offset;            // Offset value
    std::string key;         // Key associate with the publisher
};

/**
 * @brief Class containing the CAN interface node.
 * Manages subscribers and data from the csv files used in the node.
 * It also contains the functions to read the csv and parse the messages from CAN.
 */
class CanInterface : public rclcpp::Node
{
public:
 /**
     * @brief Construct the CAN interface node.
     * 
     * This initialises the CAN interface node, reads the csv files, creates the 
     * publishers from the csv, reads and parses the messages coming from the CAN buses.
     */
    CanInterface();

private:
    // Parameters to configure socketCAN.
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    int socketCan0;
    int socketCan1;

    // Threads for the two CAN buses.
    std::thread thread_0;
    std::thread thread_1;    

    // Key-vector pairs from the csv files
    std::map<std::string, std::vector<std::string>> csvdata_main;
    std::map<std::string, std::vector<std::string>> csvdata_aux;

    // Key-vector pairs of the publishers
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers;

    /**
     * @brief Create the key-value pairs from a csv file.
     * 
     * @param filepath 
     * @return std::map<std::string, std::vector<std::string>> 
     */
    std::map<std::string, std::vector<std::string>> readCSV(const std::string &filepath);

    /**
     * @brief Read the CAN messages from the specified SocketCan associated with a CAN bus.
     * 
     * @param socketCan 
     */
    void readCan(int socketCan);

    /**
     * @brief Parse the given CAN message according to the given configuration.
     * 
     * @param frame 
     * @param config 
     */
    void parseMsg(const struct can_frame& frame, const CANParseConfig& config);
};