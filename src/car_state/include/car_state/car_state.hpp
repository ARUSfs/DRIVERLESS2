#ifndef CAR_STATE_HPP
#define CAR_STATE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "common_msgs/msg/state.hpp"



class CarState : public rclcpp::Node
{
public:
    CarState();

private:
    // Callback for the subscription
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg);

    void process_data();

    // Private attributes to store received data
    float x_;
    float y_;
    float yaw_;
    float vx_;
    float vy_;
    float r_;

    bool extensometer_received_;
    bool imu_received_;

    rclcpp::Time last_time_;
    
    // Subscriber
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_extensometer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

    // Publisher for the aggregated state
    rclcpp::Publisher<common_msgs::msg::State>::SharedPtr pub_state_;
};

#endif // CAR_STATE_HPP