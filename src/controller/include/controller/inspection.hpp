/**
 * @file inspection.hpp
 * @brief Inspection mission header for ARUS Team Driverless pipeline.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/cmd.hpp"
#include "controller/speed_control.hpp"
#include "Point.h"

class Inspection : public rclcpp::Node
{
public:

    Inspection();
    
private:
    // Instances
    SpeedControl speed_control_;  
    rclcpp::Time start_time_;

    // Callbacks
    void car_state_callback(const common_msgs::msg::State::SharedPtr msg);
    void as_status_callback(const std_msgs::msg::Int16::SharedPtr msg);
    void on_timer();

    // Status AS
    int16_t as_status_ = 0x00;
    double vx_ = 0.0;       

    //Subscribers
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr as_status_sub_;

    //Timers
    rclcpp::TimerBase::SharedPtr timer_; 
    
    //Publishers
    rclcpp::Publisher<common_msgs::msg::Cmd>::SharedPtr cmd_publisher_;
};