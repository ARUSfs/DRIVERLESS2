#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP
/**
 * @file controller_node.hpp
 * 
 * @author Francis Rojas (frarojram@gmail.com).
 * 
 * @brief Controller node header for ARUS Team Driverless pipeline.
 * 
 * @date 15-11-2024
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/cmd.hpp"
#include "controller/pure_pursuit.hpp"
#include "controller/speed_control.hpp"
#include "controller/PID.hpp"
#include "Point.h"

/**
 * @brief The Controller class
 * 
 * @details Create the controller object.
 */

class Controller : public rclcpp::Node
{
public:

    Controller();
    
private:
    // Instances
    SpeedControl speedcontrol_;
    Pure_pursuit pure_pursuit_;   

    // Callbacks
    void car_state_callback(const common_msgs::msg::State::SharedPtr msg);
    void as_status_callback(const std_msgs::msg::Int16::SharedPtr msg);
    void trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
    void on_timer();
    void get_global_index(const std::vector<Point>& pointsXY_);

    // Status AS
    int16_t as_status_;

    // Variable car state
    double x_;
    double y_;
    double yaw_;
    double vx_;
    double vy_;
    double r_;
    double ax_;
    double ay_;
    double delta_;

    //Trajectory variable
    std::vector<Point> pointsXY_;  
    std::vector<float> s_;                                         
    std::vector<float> k_ ; 
    std::vector<float> speed_profile_;               
    std::vector<float> acc_profile_;
    int index_global_;         

    //Subscribers
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr as_status_sub_;
    rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr trayectory_sub_;

    //Timers
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Time previous_time_ ;
    rclcpp::Time start_time_pursuit_;
    
    // Parameters
    std::string kControllerType;
    std::string kStateTopic;
    std::string kAsStatus;
    std::string kTrajectory;
    std::string kCmdTopic;
    std::string kPursuitPointTopic;

    double kTimerFreq;
    double kLAD;
    double kTargetSpeed;
    double KP;
    double KI;
    double KD;

    rclcpp::Publisher<common_msgs::msg::Cmd>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<common_msgs::msg::PointXY>::SharedPtr pursuit_point_publisher_;
};

#endif