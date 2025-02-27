/**
 * @file new_perception_node.hpp
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Main file for the NewPerception node. 
 * Contains the main function and the implementation of the methods to achieve a robust and reliable perception algorithm for the ARUS 
 * Team, which extracts the location of the cones on the track.
 * @version 0.1
 * @date 27-2-2025
 */

#include "new_perception/new_perception_node.hpp"

NewPerception::NewPerception() : Node("NewPerception")
{

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NewPerception>());
    rclcpp::shutdown();
    return 0;
}