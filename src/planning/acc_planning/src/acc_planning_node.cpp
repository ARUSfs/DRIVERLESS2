/**
 * 
 * 
 * @file acc_planning_node.cpp
 * @author David Guil Barranco (davidguilb1@gmail.com)
 * @brief Header file for the Acc Planning node. 
 * Contains the class definition and the declaration of the methods 
 * used in the algorithm.
 * @version 0.1
 * @date 5-11-2024
 */
#include "acc_planning_node.hpp"

AccPlanning::AccPlanning() :Node("acc_planning"){

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualization>());
    rclcpp::shutdown();
    return 0;
}