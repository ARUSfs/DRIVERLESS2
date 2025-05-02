/**
 * @file epos_interface_node.hpp
 * @author Francis Rojas (frarojram@gmail.com)
 * @brief EPOS4 controller interface header for ARUS Team Driverless pipeline
 */

#include "rclcpp/rclcpp.hpp"
#include "common_msgs/msg/cmd.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "epos_interface/VSC_funtions.hpp"


class EPOS_interface : public rclcpp::Node
{
public:

    VSC epos_;

    /**
     * @brief Constructor for the EPOS_interface class.
     * Initializes the node, parameters, and subscriptions.
     */
    EPOS_interface(); 

    /**
     * @brief Destructor for the EPOS_interface class.
     * Cleans up and closes the EPOS interface.
     */
    ~EPOS_interface();

private:

    // Parameters
    int kMaxAcc;      // Maximum acceleration
    int kMaxDec;      // Maximum deceleration
    int kProfileVel;  // Profile velocity
    bool kDebug;      // Debug flag

    bool is_shutdown_;  // Flag to indicate if the node is shutting down
    double epos_pos_;   // Current position of the EPOS in radians
    double ext_pos_;    // Current position of the extensometer in radians
    double delta_cmd_;  // Commanded position in radians
    bool steer_check_;  // Flag to indicate if steering is allowed 
    bool started_;      // Flag to indicate if the movement has started
    double ext_time_;   // Time of the last extensometer reading

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    // Subscribers
    rclcpp::Subscription<common_msgs::msg::Cmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr extensometer_sub_;

    // Publishers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr steer_check_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr epos_info_pub_;

    /**
     * @brief Callback function for the timer.
     */
    void on_timer();

    /**
     * @brief Callback function for the control command subscriber.
     */
    void cmd_callback(const common_msgs::msg::Cmd::SharedPtr msg);

    /**
     * @brief Callback function for the steer extensometer subscriber.
     */
    void extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback function for the steer check subscriber.
     */
    void steer_check_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Function to disable and close the EPOS4 controller.
     */
    void clean_and_close();
};
