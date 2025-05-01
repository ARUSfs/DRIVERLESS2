/**
 * @file inspection_control_node.hpp
 * @author Álvaro Landero (alplepe02@gmail.com)
 * @brief Inspection control node header for ARUS Team Driverless pipeline.
 */

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <limits>
#include "std_msgs/msg/float32.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/cmd.hpp"
#include "inspection/PID.hpp"

class InspectionControl : public rclcpp::Node
{
public:

    InspectionControl();
    
private:
    // Variables
    PID pid_;  
    rclcpp::Time start_time_;
    int16_t as_status_ = 0x00;
    double vx_ = 0.0;   

    // Parameters
    double KP;
    double KI;
    double KD;
    double kAmplitude;
    double kFrequency;
    double kDuration;
       
    //Timers
    rclcpp::TimerBase::SharedPtr timer_; 

    //Subscribers
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr as_status_sub_;
    
    //Publishers
    rclcpp::Publisher<common_msgs::msg::Cmd>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr finish_pub_;

    /**
     * @brief Callback function for the main control timer.
     */
    void on_timer();

    /**
     * @brief Callback function for the car state subscriber.
     */
    void car_state_callback(const common_msgs::msg::State::SharedPtr msg);

    /**
     * @brief Callback function for the AS status subscriber.
     */
    void as_status_callback(const std_msgs::msg::Float32::SharedPtr msg);
};
