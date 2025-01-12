/**
 * @file epos_interface_node.hpp
 * 
 * @author Francis Rojas (frarojram@gmail.com)
 * 
 * @brief EPOS interface, Header for ARUS Team Driverless pipeline
 * 
 * @date 22-12-2024
 */

#include "epos_interface/VSC_funtions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "common_msgs/msg/cmd.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"


class EPOS_interface : public rclcpp::Node
{
public:

    VSC epos_;
    EPOS_interface(); 
    ~EPOS_interface();

private:

    void command_callback(const common_msgs::msg::Cmd::SharedPtr msg);
    void extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void range_check_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void clean_and_close();

    int MAX_ACC_;
    int MAX_DEC_;
    int PROFILE_VEL_;
    bool _is_shutdown_;
    double current_angle_;
    bool range_check_;

    rclcpp::Subscription<common_msgs::msg::Cmd>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_extensometer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_range_check_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_info_epos_;
};
