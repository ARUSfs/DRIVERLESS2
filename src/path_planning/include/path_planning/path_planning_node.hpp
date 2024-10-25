/**
 * @file path_planning_node.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @brief Header file for the Path Planning node. 
 * Contains the class definition and the declaration of the methods 
 * used in the algorithm.
 * @version 0.1
 * @date 23-10-2024
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "CDT.h"
#include <path_planning/Triangulation.h>
#include <path_planning/CDTUtils.h>
#include <common_msgs/msg/point_xy.hpp>
#include <common_msgs/msg/simplex.hpp>
#include <common_msgs/msg/triangulation.hpp>

/**
 * @brief Class containing the Path Planning node.
 * Manages subscribers and publisher for the different topics used in the algorithm.
 * It also contains the callback functions for the subscribers.
 */
class PathPlanning : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new Path Planning object
         * 
         * This initializes the Path Planning node, declares parameters, creates
         * the subscribers and publishers and contains all the necessary variables to
         * set up the algorithm.
         */
        PathPlanning();
    private:
        std::string kPerceptionTopic;
        std::string kTriangulationTopic;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;
        rclcpp::Publisher<common_msgs::msg::Triangulation>::SharedPtr triangulation_pub_;
        /**
         * @brief Callback function for the perception topic.
         * When the percetption topic recieves a message, this function is called and performs
         * all the necessary steps to generate the planning.
         * 
         * @param per_msg The point cloud received from the perception.
         */
        void perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg);
        
        /**
         * @brief Create a triangulation object from a point cloud and erase super triangle.
         * 
         * @param input_cloud 
         * @return CDT::Triangulation<double> 
         */
        CDT::Triangulation<double> create_triangulation(pcl::PointCloud<ConeXYZColorScore> input_cloud);

        /**
         * @brief Create a triangulation msg object from CDT triangulation
         * 
         * @param triangulation CDT::Triangulation<double> object containing triangulation
         * @return common_msgs::msg::Triangulation ROS2 message containing all triangulation information
         */
        common_msgs::msg::Triangulation create_triangulation_msg(CDT::Triangulation<double> triangulation);

};