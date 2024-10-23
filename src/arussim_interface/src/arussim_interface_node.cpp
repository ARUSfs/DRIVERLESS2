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

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ARUSSimInterface>());
    rclcpp::shutdown();
    return 0;
}