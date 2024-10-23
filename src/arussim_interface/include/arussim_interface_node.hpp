/**
 * @file arussim_interface_node.hpp
 * @brief ARUSSim Interface node header for ARUS Team Driverless pipeline
 */

#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/state.hpp"
#include "arussim_msgs/msg/state.hpp"


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

};