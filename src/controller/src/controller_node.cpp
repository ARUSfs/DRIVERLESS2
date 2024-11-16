/**
 * @file controller_node.cpp
 * 
 * @author Francis Rojas (frarojram@gmail.com)
 * 
 * @brief Controller node implementation for ARUS Team Driverless pipeline
 * 
 * @date 15-11-2024
 */
#include "controller/controller_node.hpp"

/**
 * @brief Constructor for the Controller class
 * 
 * @details This constructor declares all the necessary variables and
 *          instantiates all the controls required for the ART-25 to be autonomous.
 */
Controller::Controller() : Node("controller"),  speedcontrol_()
{
    this->declare_parameter<std::string>("controller_type", "pure_pursuit");
    this->declare_parameter<double>("timer_frequency", 100.0);
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
    this->get_parameter("look_ahead_distance", kLAD);

    // PID
    this->declare_parameter<double>("target", 8.0);
    this->declare_parameter<double>("KP", 43.87);
    this->declare_parameter<double>("KI", 1.29);
    this->declare_parameter<double>("KD", 0.0); 
    this->get_parameter("target", kTargetSpeed);
    this->get_parameter("KP", KP);
    this->get_parameter("KI", KI);
    this->get_parameter("KD", KD);

    speedcontrol_.get_pid().set_params(KP,KI,KD);

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

    previous_time_ = this->get_clock()->now();
}

/**
 * @brief Callback function timer of controller
 * 
 * @author Francis Rojas (frarojram@gmail.com)
 * 
 * @details Implement the control algorithm with calls to the controller libraries. 
 */  
void Controller::on_timer()
{
    if(!(pointsXY_.empty())){
        index_global_ = get_global_index(pointsXY_);
        tv_ = speed_profiles_.at(index_global_);
        ta_ = acc_profiles_.at(index_global_);

        PurePursuit::set_path(pointsXY_);
        Point position;
        position.x = x_;
        position.y = y_;
        PurePursuit::set_position(position, yaw_);
        double delta = PurePursuit::get_steering_angle(tv_/1.5);

        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - previous_time_).seconds();
        double acc = speedcontrol_.update_target(
            tv_, ta_, vx_, dt
        );
        previous_time_ = current_time;

        common_msgs::msg::Cmd cmd;       
        cmd.acc = acc;
        cmd.delta = delta;
        cmd_publisher_ -> publish(cmd); 
    }
}

/**
 * @brief get global index 
 * 
 * @author Francis Rojas (frarojram@gmail.com)
 * 
 * @details Use the global position to calculate the speed 
 * and acceleration profile at each moment. 
 */ 
int Controller::get_global_index(const std::vector<Point>& pointsXY_) {
    double min_distance_sq = std::numeric_limits<double>::max();
    int i_global_ = -1;

    for (size_t i = 0; i < pointsXY_.size(); ++i) {
            double dx = pointsXY_[i].x - x_;
            double dy = pointsXY_[i].y - y_;
            double distance_sq = dx * dx + dy * dy;

            if (distance_sq < min_distance_sq) {
                min_distance_sq = distance_sq;
                i_global_ = i;
            }
    }

        return i_global_;
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
    speed_profiles_ = msg -> speed_profile;
    acc_profiles_ = msg -> acc_profile; 
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




