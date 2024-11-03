/**
 * @file controller.hpp
 * @brief controller node header for ARUS Team Driverless pipeline
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/cmd.hpp"
#include "controller/pure_pursuit.h"
#include "controller/PID.h"
#include "Point.h"

/**
 * @class pure_pursuit
 * @brief pure_pursuit class 
 * 
 * Controller
 */

class Controller : public rclcpp::Node
{
public:
    Controller();

private:
    // Callbacks
    void car_state_callback(const common_msgs::msg::State::SharedPtr msg);
    void as_status_callback(const std_msgs::msg::Int16::SharedPtr msg);
    void trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
    void on_timer();

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
    std::vector<float> s_;                           // distance values of the points              
    std::vector<float> k_ ;                          // curvature values of the points
    std::vector<float> speed_profile_;               
    std::vector<float> acc_profile_;                

    //Subscribers
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr as_status_sub_;
    rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr trayectory_sub_;

    //Timers
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Clock::SharedPtr clock_;
    
    // Parameters
    std::string kControllerType;

    std::string kStateTopic;
    std::string kAsStatus;
    std::string kTrajectory;
    std::string kCmd;

    double kTimerFreq = 0.0;
    double look_ahead_distance_ = 0.0;
    double target_ = 0;
    double KP_ = 0.0;
    double KI_ = 0.0;
    double KD_ = 0.0;

    rclcpp::Publisher<common_msgs::msg::Cmd>::SharedPtr cmd_publisher_;
};

