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

Controller::Controller() : Node("controller"),  
                            pure_pursuit_(this->shared_from_this()),
                            pid_(this->shared_from_this())
{
    this->declare_parameter<std::string>("controller_type", "pure_pursuit");
    this->declare_parameter<double>("timer_frequency", 5.0); 

    this->get_parameter("controller_type", controller_type_);
    this->get_parameter("timer_frequency", timer_frequency_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / timer_frequency_)),
        [this]() {
            if (controller_type_ == "pure_pursuit") {

                pure_pursuit_.update_path(pointsXY_);
                double delta = pure_pursuit_.get_steering_angle();

                pid_.set_speed(vx_);
                //pid_.set_target_speed(target_speed_);

                double acc = pid_.compute_control();

                common_msgs::msg::Cmd cmd_msg;
                cmd_msg.header.stamp = this->now(); 
                cmd_msg.acc = static_cast<float>(acc); 
                cmd_msg.delta = static_cast<float>(delta); 

                cmd_publisher_->publish(cmd_msg); 
                
            }  else {
                RCLCPP_WARN(this->get_logger(), "Unknown controller type: %s", controller_type_.c_str());
            }
        }
    );

    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
        "/car_state/state", 1, std::bind(&Controller::car_state_callback, this, std::placeholders::_1));

    as_status_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/sensors/AS_status", 1, std::bind(&Controller::as_status_callback, this, std::placeholders::_1));

    trayectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        "/trajectory", 1, std::bind(&Controller::trajectory_callback, this, std::placeholders::_1));

    cmd_publisher_ = this->create_publisher<common_msgs::msg::Cmd>("/cmd_topic", 10); 
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
    pointsXY_ = msg -> points;
    s_= msg -> s;
    k_ = msg -> k;
    speed_profile_ = msg -> speed_profile;
    acc_profile_ = msg -> acc_profile; 
}

void Controller::as_status_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    as_status_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "AS_status updated: %d", as_status_);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<Controller>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}




