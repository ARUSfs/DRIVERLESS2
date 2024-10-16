/**
 * @file visualization_node.cpp
 * @brief Visualization node implementaion for ARUS Team Driverless pipeline
 */

#include "visualization/visualization_node.hpp"


/**
 * @class Visualization
 * @brief Visualization class 
 * 
 * This class represents the pipeline topics through marker messages 
 * in rviz2
 */
Visualization::Visualization() : Node("visualization")
{

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualization>());
    rclcpp::shutdown();
    return 0;
}