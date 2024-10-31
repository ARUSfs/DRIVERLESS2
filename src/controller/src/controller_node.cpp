/**
 * @file controller.cpp
 * @brief Controller node implementaion for ARUS Team Driverless pipeline
 */

#include "controller/controller_node.hpp"

/**
 * @class Controller
 * @brief Controller class 
 * 
 * 
 */

Controller::Controller() : Node("controller")
{
    this->declare_parameter<std::string>("controller_type", "pure_pursuit");
    this->declare_parameter<double>("timer_frequency", 5.0);
    this->get_parameter("controller_type", kControllerType);
    this->get_parameter("timer_frequency", kTimerFreq);

    // Topic
    this->declare_parameter<std::string>("state", "/car_state/state");
    this->declare_parameter<std::string>("as_status", "/sensors/AS_status");
    this->declare_parameter<std::string>("trajectory", "/arussim_interface/fixed_trajectory");
    this->declare_parameter<std::string>("cmd", "/controller/cmd");
    this->get_parameter("state", kStateTopic);
    this->get_parameter("as_status", kAsStatus);
    this->get_parameter("trajectory", kTrajectory);
    this->get_parameter("cmd", kCmd);

    // Pure-Pursuit
    this->declare_parameter<double>("look_ahead_distance", 6.0);
    this->get_parameter("look_ahead_distance", look_ahead_distance_);

    // PID
    this->declare_parameter<double>("target", 8.0);
    this->declare_parameter<double>("KP", 43.87);
    this->declare_parameter<double>("KI", 1.29);
    this->declare_parameter<double>("KD", 0.0); 
    this->get_parameter("target", target_);
    this->get_parameter("KP", KP_);
    this->get_parameter("KI", KI_);
    this->get_parameter("KD", KD_);

    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / kTimerFreq)),
        std::bind(&Controller::on_timer, this));

    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
        kStateTopic, 1, std::bind(&Controller::car_state_callback, this, std::placeholders::_1));

    as_status_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        kAsStatus, 1, std::bind(&Controller::as_status_callback, this, std::placeholders::_1));

    trayectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kTrajectory, 1, std::bind(&Controller::trajectory_callback, this, std::placeholders::_1));

    cmd_publisher_ = this->create_publisher<common_msgs::msg::Cmd>(kCmd, 10); 

    PID::reset(clock_);
}

void Controller::on_timer()
{
    RCLCPP_INFO(this->get_logger(), "Executing timer");
    if(!(pointsXY_.empty())){

        PurePursuit::set_path(pointsXY_);
        Point position;
        position.x = x_;
        position.y = y_;
        PurePursuit::set_position(position, yaw_);
        double delta = PurePursuit::get_steering_angle(5.0);

        double par = PID::compute_control(vx_, target_, KP_, KI_, KD_, clock_);
        double acc = par/(230*0.2);

        common_msgs::msg::Cmd cmd;       
        cmd.acc = acc;
        cmd.delta = delta;

        cmd_publisher_ -> publish(cmd); 

    }
}

void Controller::car_state_callback(const common_msgs::msg::State::SharedPtr msg)
{
    x_ = msg -> x;
    y_ = msg -> y;
    yaw_ = msg -> yaw;
    vx_ = msg -> vx;
    vy_ = msg -> vy;
    r_ = msg -> r;
    ax_ = msg -> ax;
    ay_ = msg -> ay;
    delta_ = msg -> delta;

}

void Controller::trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
{
    std::vector<common_msgs::msg::PointXY> points_common = msg -> points;
     for (const auto &pointXY : points_common) {
        Point point;
        point.x = pointXY.x;
        point.y = pointXY.y;
        pointsXY_.push_back(point);
     }

    s_= msg -> s;
    k_ = msg -> k;
    speed_profile_ = msg -> speed_profile;
    acc_profile_ = msg -> acc_profile; 
}

void Controller::as_status_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    as_status_ = msg->data;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<Controller>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}




