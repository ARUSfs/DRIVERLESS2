/**
 * @file arussim_interface_node.hpp
 * @brief ARUSSim Interface node header for ARUS Team Driverless pipeline
 */

#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/cmd.hpp"
#include "common_msgs/msg/cmd4_wd.hpp"
#include "common_msgs/msg/four_wheel_drive.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/trajectory.hpp"
#include "arussim_msgs/msg/cmd.hpp"
#include "arussim_msgs/msg/cmd4_wd.hpp"
#include "arussim_msgs/msg/four_wheel_drive.hpp"
#include "arussim_msgs/msg/trajectory.hpp"


/**
 * @class ARUSSimInterface
 * @brief ARUSSimInterface class 
 * 
 * This class translates the custom messages between the simulator and the pipeline
 */
class ARUSSimInterface : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the ARUSSimInterface class.
         */
        ARUSSimInterface();

        void cmd_callback(const common_msgs::msg::Cmd::SharedPtr msg);
        void cmd4wd_callback(const common_msgs::msg::Cmd4WD::SharedPtr msg);
        void wheel_speeds_callback(const arussim_msgs::msg::FourWheelDrive::SharedPtr msg);
        void trajectory_callback(const arussim_msgs::msg::Trajectory::SharedPtr msg);
    
    private:

        rclcpp::Publisher<arussim_msgs::msg::Cmd>::SharedPtr cmd_pub_;
        rclcpp::Publisher<arussim_msgs::msg::Cmd4WD>::SharedPtr cmd4wd_pub_;
        rclcpp::Publisher<common_msgs::msg::FourWheelDrive>::SharedPtr wheel_speeds_pub_;
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

        rclcpp::Subscription<common_msgs::msg::Cmd>::SharedPtr cmd_sub_;
        rclcpp::Subscription<common_msgs::msg::Cmd4WD>::SharedPtr cmd4wd_sub_;
        rclcpp::Subscription<arussim_msgs::msg::FourWheelDrive>::SharedPtr wheel_speeds_sub_;
        rclcpp::Subscription<arussim_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;

};