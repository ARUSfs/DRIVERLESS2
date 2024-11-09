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
#include "acc_planning/acc_planning_node.hpp"

AccPlanning::AccPlanning() :Node("acc_planning"){


perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
   "/arussim/perception", 10, std::bind(&AccPlanning::perception_callback, this, std::placeholders::_1));

}

void AccPlanning::perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr per_msg)
{   
    pcl::PointCloud<ConeXYZColorScore> pcl_cloud;
    pcl::fromROSMsg(*per_msg, pcl_cloud);
    RCLCPP_INFO(this->get_logger(), "Callback ejecutado: nube de puntos recibida con %lu puntos.", pcl_cloud.size());


}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccPlanning>());
    rclcpp::shutdown();
    return 0;
}