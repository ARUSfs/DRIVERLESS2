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

class CanInterface : public rclcpp::Node
{

public:
    CanInterface();

private :
    std::map<std::string, std::vector<std::string>> csvdata_main;
    std::map<std::string, std::vector<std::string>> csvdata_aux;

    std::map<std::string, std::vector<std::string>> readCSV(const std::string &filepath);

    void pubHeartBeat();
    void controlsCallback(common_msgs::msg::Cmd);
    void brakeLightCallback(std_msgs::msg::Int16);
    void steeringInfoCallback(std_msgs::msg::Float32MultiArray);
    void ASStatusCallback(std_msgs::msg::Int16);
    void lapCounterCallback(std_msgs::msg::Int16);
    void conesCountCallback(sensor_msgs::msg::PointCloud2);
    void conesCountAllCallback(sensor_msgs::msg::PointCloud2);
    void targetSpeedCallback(std_msgs::msg::Float32);
    void DL500Callback();
    void DL501Callback();
    void DL502Callback();
    void pcTempCallback();
    
    struct CANParseConfig {
        uint8_t startByte;       // Start byte position
        uint8_t endByte;         // End byte position
        std::string endianness;  // LE or BE
        float scale;             // Scaling factor
        float offset;            // Offset value
        std::string target;      // Target variable name
        std::string type;        // Type of the target variable
    };

    void readCan(int);
    void parseMsg(const struct can_frame& frame, const CANParseConfig& config);

    void readCan1();
    void readCan0();

    int socketCan0;
    int socketCan1;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    rclcpp::Subscription<common_msgs::msg::Cmd>::SharedPtr controlsSub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr ASStatusSub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr steeringInfoSub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lapCounterSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr conesCountSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr conesCountAllSub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr targetSpeedSub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr brakeLightSub;

   
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motorSpeedPub;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ASStatusPub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr GPSPub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr GPSSpeedPub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr IMUPub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steeringAnglePub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr RESRangePub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr PCTempPub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr DL500Pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr DL501Pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr DL502Pub;

    rclcpp::TimerBase::SharedPtr pcTempTimer;
    rclcpp::TimerBase::SharedPtr heartBeatTimer;
    rclcpp::TimerBase::SharedPtr DL500Timer;
    rclcpp::TimerBase::SharedPtr DL501Timer;
    rclcpp::TimerBase::SharedPtr DL502Timer;


    float pc_temp;
    void getPcTemp();

    uint8_t actual_speed;

    uint8_t target_speed;
    int8_t actual_steering_angle;
    int8_t target_steering_angle;
    uint8_t brake_hydr_actual;
    uint8_t brake_hydr_target;
    uint8_t pneumatic_press;
    int8_t motor_moment_actual;
    int8_t motor_moment_target;

    sensor_msgs::msg::Imu IMUData;
    sensor_msgs::msg::NavSatFix GPSData;
    geometry_msgs::msg::Vector3 GPSVelData;
    std_msgs::msg::Float32 steeringAngleData;
    std_msgs::msg::Float32 RESData;
    std_msgs::msg::Float32 InvSpeedData;
    
    uint8_t AS_state;
    uint8_t EBS_state;
    uint8_t AMI_state;
    bool steering_state;
    uint8_t service_brake_state;
    uint8_t lap_counter;
    uint8_t cones_count_actual;
    uint16_t cones_count_all;

    void parseInvSpeed(unsigned char []);
    void parseMission(unsigned char []);
    void parseASStatus(unsigned char []);
    void parseAcc(unsigned char []);
    void parseEulerAngles(unsigned char []);
    void parseAngularVelocity(unsigned char []);
    void parseGPS(unsigned char []);
    void parseGPSVel(unsigned char []);
    void parseSteeringAngle(unsigned char []);
    void parseRES(unsigned char []);
    void parseBrakeHydr(unsigned char []);
    void parsePneumatic(unsigned char []);
    void initialize_timer();

    std::thread thread_0;
    std::thread thread_1;    
};