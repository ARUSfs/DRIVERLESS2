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

    void cmd_callback(const common_msgs::msg::Cmd::SharedPtr msg);
    void extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void steer_check_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void on_timer();
    void clean_and_close();

    int MAX_ACC_;
    int MAX_DEC_;
    int PROFILE_VEL_;
    bool is_shutdown_;
    double epos_pos_;
    double ext_pos_;
    double delta_cmd_;
    bool steer_check_;
    double ext_time_;

    rclcpp::Subscription<common_msgs::msg::Cmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr extensometer_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr steer_check_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr epos_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
