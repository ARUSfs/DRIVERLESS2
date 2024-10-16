#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/point_xy.hpp"

class Controller : public rclcpp::Node
{
public:
    Controller();

private:
    // Callbacks
    void car_state_callback(const common_msgs::msg::State::SharedPtr msg);
    void as_status_callback(const std_msgs::msg::Int16::SharedPtr msg);
    void trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
    void pp_callback();

    // Status AS
    int16_t as_status_;

    // Variable car state
    double x_;
    double y_;

    double ax_;
    double ay_;

    double vx_;
    double vy_;

    double r_;
    double yaw_;
    double delta_;

    //Trajectory variable
    std::vector<common_msgs::msg::PointXY> pointsXY_;  //points on the trajectory
    std::vector<float> s_;                           // distance values of the points              
    std::vector<float> k_ ;                          // curvature values of the points
    std::vector<float> speed_profile_;               // speed profile of the trajectory
    std::vector<float> acc_profile_;              //acceleration profile of the trajectory   

    //Subscribers
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr as_status_sub_;
    rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr trayectory_sub_;

    //Timers
    rclcpp::TimerBase::SharedPtr timer_;  // Timer for executing control logic
    
    // Parameters for control type and frequency
    std::string controller_type_;
    double timer_frequency_;
};

#endif  // CONTROLLER_HPP

