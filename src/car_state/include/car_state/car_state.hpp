/**
 * @file car_state.hpp
 * @brief CarState node header for ARUS Team Driverless pipeline
 */

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/four_wheel_drive.hpp"

/**
 * @class CarState
 * @brief CarState class 
 * 
 * Header node car_state
 * 
 */

class CarState : public rclcpp::Node
{
public:
    CarState();

private:
    // Callback for the subscription
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void wheel_speeds_callback(const common_msgs::msg::FourWheelDrive::SharedPtr msg);
    void arussim_ground_truth_callback(const common_msgs::msg::State::SharedPtr msg);

    // Funtions
    void on_timer();

    // Private attributes to store received data
    double x_=0;
    double y_=0;
    double yaw_ = 0;
    double vx_ = 0;
    double vy_ = 0;
    double r_ = 0;
    double ax_ = 0;
    double ay_ = 0;
    double delta_ = 0;


    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_extensometer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<common_msgs::msg::FourWheelDrive>::SharedPtr sub_wheel_speeds_;
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr sub_arussim_ground_truth_;

    // Publisher for the aggregated state
    rclcpp::Publisher<common_msgs::msg::State>::SharedPtr pub_state_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};