/**
 * @file arussim_interface_node.cpp
 * @brief ARUSSimInterface node implementaion for ARUS Team Driverless pipeline
 */

#include "arussim_interface/arussim_interface_node.hpp"


/**
 * @class ARUSSimInterface
 * @brief ARUSSimInterface class 
 * 
 * This class translates the custom messages between the simulator and the pipeline
 */
ARUSSimInterface::ARUSSimInterface() : Node("arussim_interface")
{
    this->get_config_parameters();

    cmd_pub_ = this->create_publisher<arussim_msgs::msg::Cmd>(
        kSimCmdTopic, 10);
    cmd4wd_pub_ = this->create_publisher<arussim_msgs::msg::Cmd4WD>(
        kSimCmd4WDTopic, 10);
    wheel_speeds_pub_ = this->create_publisher<common_msgs::msg::FourWheelDrive>(
        kPipelineWheelSpeedsTopic, 10);
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>(
        kPipelineTrajectoryTopic, 10);
    ground_truth_pub_ = this->create_publisher<common_msgs::msg::State>(
        kPipelineGroundTruthTopic, 10);

    cmd_sub_ = this->create_subscription<common_msgs::msg::Cmd>(
        kPipelineCmdTopic, 10, 
        std::bind(&ARUSSimInterface::cmd_callback, this, std::placeholders::_1));
    cmd4wd_sub_ = this->create_subscription<common_msgs::msg::Cmd4WD>(
        kPipelineCmd4WDTopic, 10, 
        std::bind(&ARUSSimInterface::cmd4wd_callback, this, std::placeholders::_1));
    sim_wheel_speeds_sub_ = this->create_subscription<arussim_msgs::msg::FourWheelDrive>(
        kSimWheelSpeedsTopic, 10, 
        std::bind(&ARUSSimInterface::sim_wheel_speeds_callback, this, std::placeholders::_1));
    sim_trajectory_sub_ = this->create_subscription<arussim_msgs::msg::Trajectory>(
        kSimTrajectoryTopic, 10, 
        std::bind(&ARUSSimInterface::sim_trajectory_callback, this, std::placeholders::_1));
    sim_state_sub_ = this->create_subscription<arussim_msgs::msg::State>(
        kSimStateTopic, 10, 
        std::bind(&ARUSSimInterface::sim_state_callback, this, std::placeholders::_1));
     
}

void ARUSSimInterface::cmd_callback(const common_msgs::msg::Cmd::SharedPtr msg)
{
    arussim_msgs::msg::Cmd cmd_msg;
    cmd_msg.header = msg->header;
    cmd_msg.acc = msg->acc;
    cmd_msg.delta = msg->delta;
    cmd_pub_->publish(cmd_msg);
}

void ARUSSimInterface::cmd4wd_callback(const common_msgs::msg::Cmd4WD::SharedPtr msg)
{
    arussim_msgs::msg::Cmd4WD cmd4wd_msg;
    cmd4wd_msg.header = msg->header;
    cmd4wd_msg.acc.front_right = msg->acc.front_right;
    cmd4wd_msg.acc.front_left = msg->acc.front_left;
    cmd4wd_msg.acc.rear_right = msg->acc.rear_right;
    cmd4wd_msg.acc.rear_left = msg->acc.rear_left;
    cmd4wd_pub_->publish(cmd4wd_msg);
}

void ARUSSimInterface::sim_wheel_speeds_callback(const arussim_msgs::msg::FourWheelDrive::SharedPtr msg)
{
    common_msgs::msg::FourWheelDrive wheel_speeds_msg;
    wheel_speeds_msg.front_right = msg->front_right;
    wheel_speeds_msg.front_left = msg->front_left;
    wheel_speeds_msg.rear_right = msg->rear_right;
    wheel_speeds_msg.rear_left = msg->rear_left;
    wheel_speeds_pub_->publish(wheel_speeds_msg);
}

void ARUSSimInterface::sim_trajectory_callback(const arussim_msgs::msg::Trajectory::SharedPtr msg)
{
    common_msgs::msg::Trajectory trajectory_msg;
    trajectory_msg.header = msg->header;
    for (int i = 0; i < msg->points.size(); i++){
        common_msgs::msg::PointXY point;
        point.x = msg->points[i].x;
        point.y = msg->points[i].y;
        trajectory_msg.points.push_back(point);
    }
    trajectory_msg.speed_profile = msg -> speed_profile;
    trajectory_msg.acc_profile = msg -> acc_profile;
    trajectory_pub_->publish(trajectory_msg);
}

void ARUSSimInterface::sim_state_callback(const arussim_msgs::msg::State::SharedPtr msg)
{
    common_msgs::msg::State state_msg;
    state_msg.header = msg->header;
    state_msg.x = msg->x;
    state_msg.y = msg->y;
    state_msg.yaw = msg->yaw;
    state_msg.vx = msg->vx;
    state_msg.vy = msg->vy;
    state_msg.r = msg->r;
    state_msg.ax = msg->ax;
    state_msg.ay = msg->ay;
    state_msg.delta = msg->delta;
    ground_truth_pub_->publish(state_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ARUSSimInterface>());
    rclcpp::shutdown();
    return 0;
}