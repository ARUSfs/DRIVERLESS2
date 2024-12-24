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

struct CANParseConfig {
    uint8_t startByte;       // Start byte position
    uint8_t endByte;         // End byte position
    float scale;             // Scaling factor
    float offset;            // Offset value
    std::string key;         // Key associate with the publisher
};

class CanInterface : public rclcpp::Node
{

public:
    CanInterface();

private:
    std::map<std::string, std::vector<std::string>> csvdata_main;
    std::map<std::string, std::vector<std::string>> csvdata_aux;
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers;

    std::map<std::string, std::vector<std::string>> readCSV(const std::string &filepath);

    void readCan(int);
    void parseMsg(const struct can_frame& frame, const CANParseConfig& config);

    void readCan1();
    void readCan0();

    int socketCan0;
    int socketCan1;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    std::thread thread_0;
    std::thread thread_1;    
};