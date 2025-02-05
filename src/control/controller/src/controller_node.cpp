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
#include <limits>

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
    this->declare_parameter<std::string>("trajectory", "/path_planning/trajectory");
    this->declare_parameter<std::string>("cmd", "/controller/cmd");
    this->declare_parameter<std::string>("pursuit_point", "/controller/pursuit_point");
    this->declare_parameter<std::string>("target_speed", "/controller/target_speed");
    this->get_parameter("state", kStateTopic);
    this->get_parameter("trajectory", kTrajectoryTopic);
    this->get_parameter("cmd", kCmdTopic);
    this->get_parameter("pursuit_point",kPursuitPointTopic);
    this->get_parameter("target_speed", kTargetSpeedTopic);

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

    // Cmd limits
    this->declare_parameter<double>("min_cmd", 0.0);
    this->declare_parameter<double>("max_cmd", 0.1);
    this->get_parameter("min_cmd", kMinCmd);
    this->get_parameter("max_cmd", kMaxCmd);

    speed_control_.pid_.set_params(KP,KI,KD);

    previous_time_ = this->get_clock()->now();

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / kTimerFreq)),
        std::bind(&Controller::on_timer, this));

    // Subscribers
    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
        kStateTopic, 1, std::bind(&Controller::car_state_callback, this, std::placeholders::_1));
    run_check_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/car_state/run_check", 1, std::bind(&Controller::run_check_callback, this, std::placeholders::_1));
    trayectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kTrajectoryTopic, 1, std::bind(&Controller::trajectory_callback, this, std::placeholders::_1));
    final_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        "/path_planning/final_trajectory", 1, std::bind(&Controller::trajectory_callback, this, std::placeholders::_1));
    // optimized_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
    //     "/trajectory_optimization/trajectory", 1, std::bind(&Controller::trajectory_callback, this, std::placeholders::_1));


    // Publishers
    cmd_pub_ = this->create_publisher<common_msgs::msg::Cmd>(kCmdTopic, 10);
    pursuit_point_pub_ = this->create_publisher<common_msgs::msg::PointXY>(kPursuitPointTopic, 10);
    target_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(kTargetSpeedTopic, 10);
}

/**
 * @brief Callback function timer of controller
 * 
 * @details Implement the control algorithm with calls to the controller libraries. 
 */  
void Controller::on_timer()
{
    if(!(pointsXY_.empty()) && run_check_){
        get_global_index();

        double target_speed = kTargetSpeed;
        if(!(speed_profile_.empty())){
            target_speed = speed_profile_.at(index_global_);
        }
        std_msgs::msg::Float32 target_speed_msg;
        target_speed_msg.data = target_speed;
        target_speed_pub_->publish(target_speed_msg);

        double target_acc = 0.0;
        if(!(acc_profile_.empty())){
            target_acc = acc_profile_.at(index_global_);
        }

        pure_pursuit_.set_path(pointsXY_);
        Point position;
        position.x = x_;
        position.y = y_;
        pure_pursuit_.set_position(position, yaw_);
        auto [delta, pursuit_point] = pure_pursuit_.get_steering_angle(index_global_, kLAD);
        common_msgs::msg::PointXY pursuit_point_msg;
        pursuit_point_msg.x = pursuit_point.x;
        pursuit_point_msg.y = pursuit_point.y;
        pursuit_point_pub_ -> publish(pursuit_point_msg);

        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - previous_time_).seconds();
        double acc = speed_control_.get_acc_command(target_speed, target_acc, vx_, dt);
        previous_time_ = current_time;

        common_msgs::msg::Cmd cmd;       
        cmd.acc = std::clamp(acc, kMinCmd, kMaxCmd);
        cmd.delta = delta;
        cmd_pub_ -> publish(cmd); 
    }
}

/**
 * @brief get global index of the vehicle in the trajectory
 * @details Use the global position to calculate the speed 
 * and acceleration profile at each moment. 
 */ 
void Controller::get_global_index() {
    double min_dist = std::numeric_limits<double>::max();
    int i_global = -1;
    int N = pointsXY_.size();
    int i = 0;

    
    // If the trajectory is not updated, the search starts from the last index
    if(!new_trajectory_){
        i = index_global_; 
    }
    new_trajectory_ = false;

    // Search for the closest point from the current position
    // N+10 and i%N is added to allow restarting loops (trackdrive)
    while (i < N + 10) {
        double dx = pointsXY_[i % N].x - x_;
        double dy = pointsXY_[i % N].y - y_;
        double dist = dx * dx + dy * dy;

        if (dist < min_dist) {
            min_dist = dist;
            i_global = i % N;
        } else if(!new_trajectory_ && dist > 5.0){ 
            // If the distance is greater than 5 meters from last index, 
            // the search is stopped to avoid errors due to loops in the trajectory (skidpad)
            break;
        }
        i++;
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
    // Check if the trajectory is the same
    if(msg->points.size() != pointsXY_.size()
       || msg->points[msg->points.size()-1].x != pointsXY_[pointsXY_.size()-1].x){
        new_trajectory_ = true;
    }

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

void Controller::run_check_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    run_check_ = msg -> data;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<Controller>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}




