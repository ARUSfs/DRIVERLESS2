#include "controller/controller.hpp"

Controller::Controller() : Node("controller_node")
{
    // Declare parameters (can be set from launch or config file)
    this->declare_parameter<std::string>("controller_type", "pure_pursuit");
    this->declare_parameter<double>("timer_frequency", 5.0);  // Default frequency 10 Hz

    // Retrieve parameters from configuration or launch file
    this->get_parameter("controller_type", controller_type_);
    this->get_parameter("timer_frequency", timer_frequency_);

    // Configure timer once in the constructor based on the selected controller and frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / timer_frequency_)),
        [this]() {
            // Execute the appropriate callback based on the selected controller
            if (controller_type_ == "pure_pursuit") {
                pp_callback();
            }  else {
                RCLCPP_WARN(this->get_logger(), "Unknown controller type: %s", controller_type_.c_str());
            }
        });

    // Subscription to car state
    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
        "/car_state/state", 1, std::bind(&Controller::car_state_callback, this, std::placeholders::_1));

    // Subscription to 'can/AS_status'
    as_status_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/sensors/AS_status", 1, std::bind(&Controller::as_status_callback, this, std::placeholders::_1));

    trayectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        "/trajectory", 1, std::bind(&Controller::trajectory_callback, this, std::placeholders::_1));
}

void Controller::car_state_callback(const common_msgs::msg::State::SharedPtr msg)
{
    x_ = msg -> x;
    y_ = msg -> y;

    vx_ = msg -> vx;
    vy_ = msg -> vy;

    ax_ = msg -> ax;
    ay_ = msg -> ay;

    yaw_ = msg -> yaw;
    r_ = msg -> r;
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

void Controller::pp_callback()
{
    RCLCPP_INFO(this->get_logger(), "Executing Pure Pursuit control.");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<Controller>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}




