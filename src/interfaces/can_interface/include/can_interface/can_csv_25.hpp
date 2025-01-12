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
#include "std_msgs/msg/bool.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "common_msgs/msg/car_info.hpp"
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
#include <cerrno>    

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
    std::map<std::string, std::vector<std::string>> read_csv(const std::string &filepath);

    /**
     * @brief Read the CAN messages from the specified SocketCan associated with a CAN bus.
     * 
     * @param socketCan 
     */
    void read_CAN(int socketCan);

    /**
     * @brief Parse the given CAN message according to the given configuration.
     * 
     * @param frame 
     * @param config 
     */
    void parse_msg(const struct can_frame& frame, const CANParseConfig& config);

    /**
     * @brief Filter the message if the subID matches the criteria in the csv
     * 
     * @param frame 
     * @param aux_vector_subID 
     */
    bool filter_subID(const struct can_frame& frame, const std::string& aux_vector_subID);

    // CALLBACKS

    void pubHeartBeat();
    void controlsCallback(common_msgs::msg::Cmd);
    void brakeLightCallback(std_msgs::msg::Int16);
    void steeringInfoCallback(std_msgs::msg::Float32MultiArray);
    void ASStatusCallback(std_msgs::msg::Int16);
    void targetSpeedCallback(std_msgs::msg::Float32);
    void dl_timer_callback();
    void send_dl500();
    void send_dl501();
    void send_dl502();
    void car_info_callback(common_msgs::msg::CarInfo);
    void run_check_callback(std_msgs::msg::Bool);

    // SUBS

    rclcpp::Subscription<common_msgs::msg::Cmd>::SharedPtr control_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr ASStatusSub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr steeringInfoSub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lapCounterSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr conesCountSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr conesCountAllSub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr targetSpeedSub;
    rclcpp::Subscription<common_msgs::msg::CarInfo>::SharedPtr car_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_check_sub_;

    // TIMERS

    rclcpp::TimerBase::SharedPtr heartBeatTimer;
    rclcpp::TimerBase::SharedPtr dl_timer_;

    float pc_temp;
    void getPcTemp();

    // VAR

    float speed_actual_ = 0;
    float speed_target_ = 0;
    float steering_angle_actual_ = 0;
    float steering_angle_target_ = 0;
    float brake_hydr_actual_ = 0;
    float brake_hydr_target_ = 0;
    float motor_moment_actual_ = 0;
    float motor_moment_target_ = 0;

    float ax_ = 0;
    float ay_ = 0;
    float yaw_rate_ = 0;

    int as_status_ = 0;
    int asb_ebs_state_ = 0;
    int ami_state_ = 0;
    int steering_state_ = 0;
    int asb_redundancy_state_ = 0;
    int lap_counter_ = 0;
    int cones_count_actual_ = 0;
    int cones_count_all_ = 0;

    bool run_check_ = false;
};