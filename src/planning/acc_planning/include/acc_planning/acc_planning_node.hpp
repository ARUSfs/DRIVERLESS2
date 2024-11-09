/**
 * 
 * 
 * @file acc_planning_node.hpp
 * @author David Guil Barranco (davidguilb1@gmail.com)
 * @brief Header file for the Acc Planning node. 
 * Contains the class definition and the declaration of the methods 
 * used in the algorithm.
 * @version 0.1
 * @date 5-11-2024
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "acc_planning/ConeXYZColorScore.h"



class AccPlanning: public rclcpp::Node{
public:
   
    AccPlanning();
private:
        std::string kPerceptionTopic;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;
        /**
         * @brief Callback function for the perception topic.
         * When the percetption topic recieves a message, this function is called and performs
         * all the necessary steps to generate the planning.
         * 
         * @param per_msg The point cloud received from the perception.
         */
        void perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg);
};

