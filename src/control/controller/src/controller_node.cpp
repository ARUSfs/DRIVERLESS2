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
 * @details This constructor declares all the necessary variables and
 *          instantiates all the controls required for the ART-25 to be autonomous.
 */
Controller::Controller() : Node("controller"),  
    speed_control_(),
    pure_pursuit_()
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
    this->declare_parameter<std::string>("pursuit_point", "/controller/pursuit_point");
    this->get_parameter("state", kStateTopic);
    this->get_parameter("as_status", kAsStatus);
    this->get_parameter("trajectory", kTrajectory);
    this->get_parameter("cmd", kCmdTopic);
    this->get_parameter("pursuit_point",kPursuitPointTopic);

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

    speed_control_.pid_.set_params(KP,KI,KD);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / kTimerFreq)),
        std::bind(&Controller::on_timer, this));

    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
        kStateTopic, 1, std::bind(&Controller::car_state_callback, this, std::placeholders::_1));

    as_status_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        kAsStatus, 1, std::bind(&Controller::as_status_callback, this, std::placeholders::_1));

    trayectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kTrajectory, 1, std::bind(&Controller::trajectory_callback, this, std::placeholders::_1));

    cmd_publisher_ = this->create_publisher<common_msgs::msg::Cmd>(kCmdTopic, 10);

    pursuit_point_publisher_ = this->create_publisher<common_msgs::msg::PointXY>(kPursuitPointTopic, 10);

    previous_time_ = this->get_clock()->now();
}

/**
 * @brief Callback function timer of controller
 * 
 * @details Implement the control algorithm with calls to the controller libraries. 
 */  
void Controller::on_timer()
{
    if(!(pointsXY_.empty())){
        get_global_index(pointsXY_);

        double target_speed = kTargetSpeed;
        if(!(speed_profile_.empty())){
            target_speed = speed_profile_.at(index_global_);
        }

        double target_acc = 0.0;
        if(!(acc_profile_.empty())){
            target_acc = acc_profile_.at(index_global_);
        }

        pure_pursuit_.set_path(pointsXY_);
        Point position;
        position.x = x_;
        position.y = y_;
        pure_pursuit_.set_position(position, yaw_);
        auto [delta, pursuit_point] = pure_pursuit_.get_steering_angle(index_global_,target_speed / 1.5);
        common_msgs::msg::PointXY pursuit_point_msg;
        pursuit_point_msg.x = pursuit_point.x;
        pursuit_point_msg.y = pursuit_point.y;
        pursuit_point_publisher_ -> publish(pursuit_point_msg);

        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - previous_time_).seconds();
        double acc = speed_control_.get_acc_command(target_speed, target_acc, vx_, dt);
        previous_time_ = current_time;

        common_msgs::msg::Cmd cmd;       
        cmd.acc = acc;
        cmd.delta = delta;
        cmd_publisher_ -> publish(cmd); 
    }
}

/**
 * @brief get global index of the vehicle in the trajectory
 * @details Use the global position to calculate the speed 
 * and acceleration profile at each moment. 
 */ 
void Controller::get_global_index(const std::vector<Point>& pointsXY_) {
    double min_distance_sq = std::numeric_limits<double>::max();
    int i_global = -1;

    for (size_t i = 0; i < pointsXY_.size(); ++i) {
            double dx = pointsXY_[i].x - x_;
            double dy = pointsXY_[i].y - y_;
            double distance_sq = dx * dx + dy * dy;

            if (distance_sq < min_distance_sq) {
                min_distance_sq = distance_sq;
                i_global = i;
            }
    }

    index_global_ = i_global;
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
    pointsXY_.clear();
    s_.clear();
    k_.clear();
    speed_profile_.clear();
    acc_profile_.clear();

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




