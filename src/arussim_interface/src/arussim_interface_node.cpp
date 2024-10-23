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
    cmd_pub_ = this->create_publisher<arussim_msgs::msg::Cmd>("/arussim/cmd", 10);
    cmd4wd_pub_ = this->create_publisher<arussim_msgs::msg::Cmd4WD>("/arussim/cmd4wd", 10);
    wheel_speeds_pub_ = this->create_publisher<common_msgs::msg::FourWheelDrive>("/can/wheel_speeds", 10);
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>("/path_planning/trajectory", 10);

    cmd_sub_ = this->create_subscription<common_msgs::msg::Cmd>("/control/cmd", 10, std::bind(&ARUSSimInterface::cmd_callback, this, std::placeholders::_1));
    cmd4wd_sub_ = this->create_subscription<common_msgs::msg::Cmd4WD>("/control/cmd4wd", 10, std::bind(&ARUSSimInterface::cmd4wd_callback, this, std::placeholders::_1));
    wheel_speeds_sub_ = this->create_subscription<arussim_msgs::msg::FourWheelDrive>("/arussim/wheel_speeds", 10, std::bind(&ARUSSimInterface::wheel_speeds_callback, this, std::placeholders::_1));
    trajectory_sub_ = this->create_subscription<arussim_msgs::msg::Trajectory>("/arussim/trajectory", 10, std::bind(&ARUSSimInterface::trajectory_callback, this, std::placeholders::_1));
     
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

void ARUSSimInterface::wheel_speeds_callback(const arussim_msgs::msg::FourWheelDrive::SharedPtr msg)
{
    common_msgs::msg::FourWheelDrive wheel_speeds_msg;
    wheel_speeds_msg.front_right = msg->front_right;
    wheel_speeds_msg.front_left = msg->front_left;
    wheel_speeds_msg.rear_right = msg->rear_right;
    wheel_speeds_msg.rear_left = msg->rear_left;
    wheel_speeds_pub_->publish(wheel_speeds_msg);
}

void ARUSSimInterface::trajectory_callback(const arussim_msgs::msg::Trajectory::SharedPtr msg)
{
    common_msgs::msg::Trajectory trajectory_msg;
    trajectory_msg.header = msg->header;
    for (int i = 0; i < msg->points.size(); i++){
        common_msgs::msg::PointXY point;
        point.x = msg->points[i].x;
        point.y = msg->points[i].y;
        trajectory_msg.points.push_back(point);
    }
    trajectory_pub_->publish(trajectory_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ARUSSimInterface>());
    rclcpp::shutdown();
    return 0;
}