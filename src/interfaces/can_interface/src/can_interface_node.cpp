/**
 * @file can_interface_node.cpp
 * @brief CanInterface node implementaion for ARUS Team Driverless pipeline
 */

#include "can_interface/can_interface_node.hpp"


/**
 * @class CanInterface
 * @brief CanInterface class 
 * 
 * Text here
 */
CanInterface::CanInterface() : Node("can_interface")
{
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanInterface>());
    rclcpp::shutdown();
    return 0;
}
