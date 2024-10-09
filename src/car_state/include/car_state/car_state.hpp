#ifndef CAR_STATE_HPP
#define CAR_STATE_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"


class CarState : public rclcpp::Node
{
public:
    CarState();

private:
    // Callback for the subscription
    void sim_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void can_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void imu_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void process_data();

    // Private attributes to store received data
    float x_;
    float y_;
    float yaw_;
    float vx_;
    float vy_;
    float r_;
    
    // Subscriber
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_sim_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_can_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_imu_;
};

#endif // CAR_STATE_HPP