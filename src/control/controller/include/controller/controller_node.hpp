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
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/cmd.hpp"
#include "controller/pure_pursuit.hpp"
#include "controller/speed_control.hpp"
#include "controller/PID.hpp"
#include "Point.h"
#include "lti_mpc.hpp"

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
    SpeedControl speed_control_;
    Pure_pursuit pure_pursuit_;   
    LtiMpc lti_mpc_;

    // Callbacks
    void car_state_callback(const common_msgs::msg::State::SharedPtr msg);
    void trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
    void optimized_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
    void run_check_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void on_speed_timer();
    void on_steer_timer();
    void get_global_index();

    bool run_check_ = false;
    bool optimized_ = false;

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

    double v_delta_;
    double prev_delta_;

    double acc_cmd_;
    double delta_cmd_;

    //Trajectory variable
    std::vector<Point> pointsXY_;  
    std::vector<float> s_;                                         
    std::vector<float> k_ ; 
    std::vector<float> speed_profile_;               
    std::vector<float> acc_profile_;
    int index_global_ = 0;      
    bool new_trajectory_ = false;         

    //Subscribers
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
    rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr trayectory_sub_;
    rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr optimized_trajectory_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_check_sub_;

    //Timers
    rclcpp::TimerBase::SharedPtr speed_timer_;
    rclcpp::TimerBase::SharedPtr steer_timer_;
    rclcpp::Time previous_time_ ;
    
    // Parameters
    std::string kFirstLapSteerControl;
    std::string kOptimizedSteerControl;
    std::string kStateTopic;
    std::string kTrajectoryTopic;
    std::string kCmdTopic;
    std::string kPursuitPointTopic;
    std::string kTargetSpeedTopic;

    double kSpeedTimerFreq;
    double kSteerTimerFreq;
    double kLAD;
    double kTargetSpeed;
    double KP;
    double KI;
    double KD;
    double kMinCmd;
    double kMaxCmd;
    double kMaxSteer;
    double kCostLateralDeviation;
    double kCostAngularDeviation;
    double kCostSteeringDelta;

    rclcpp::Publisher<common_msgs::msg::Cmd>::SharedPtr cmd_pub_;
    rclcpp::Publisher<common_msgs::msg::PointXY>::SharedPtr pursuit_point_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_speed_pub_;
};

#endif