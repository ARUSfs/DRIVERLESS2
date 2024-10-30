/**
 * @file controller.cpp
 * @brief Controller node implementaion for ARUS Team Driverless pipeline
 */

#include "controller/controller.hpp"

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

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / kTimerFreq)),
        std::bind(&Controller::on_timer, this));

    car_state_sub_ = this->create_subscription<arussim_msgs::msg::State>(
        "/arussim/state", 1, std::bind(&Controller::car_state_callback, this, std::placeholders::_1));

    as_status_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/sensors/AS_status", 1, std::bind(&Controller::as_status_callback, this, std::placeholders::_1));

    trayectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        "/arussim_interface/fixed_trajectory", 1, std::bind(&Controller::trajectory_callback, this, std::placeholders::_1));

    cmd_publisher_ = this->create_publisher<common_msgs::msg::Cmd>("/controller/cmd", 10); 
}

void Controller::on_timer()
{
    RCLCPP_INFO(this->get_logger(), "Executing timer");
    if(!(pointsXY_.empty())){
        PurePursuit::set_path(pointsXY_);

        common_msgs::msg::PointXY position;
        position.x = x_;
        position.y = y_;
        PurePursuit::set_position(position, yaw_);

        double delta = PurePursuit::get_steering_angle(5.0);
        // RCLCPP_INFO(this->get_logger(), "Delta: %f", delta);

        auto current_time = std::chrono::steady_clock::now();        
        double par = PID::compute_control(vx_, 2, 43.87, 1.29, 0, current_time);
        double acc = par/(240*0.2);

        common_msgs::msg::Cmd cmd;       
        cmd.acc = acc;
        cmd.delta = delta;
        cmd_publisher_ -> publish(cmd); 
        RCLCPP_INFO(this->get_logger(), "Delta: %f", cmd.acc);

    }
}

void Controller::car_state_callback(const arussim_msgs::msg::State::SharedPtr msg)
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
    pointsXY_ = msg -> points;
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




