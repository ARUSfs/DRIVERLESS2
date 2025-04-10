/**
 * @file new_perception_node.hpp
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Header file for the NewPerception node.
 * Contains the class definition and the declaration of the methods 
 * used in the algorithm. 
 * @version 0.1
 * @date 27-2-2025
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>

#include "new_perception/cropping.h"

/**
 * @class NewPerception
 * @brief Class containing the NewPerception node.
 * Manages subscribers and publisher for the different topics used in the algorithm.
 * It also contains the callback functions for the subscribers.
 */
class NewPerception : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a NewPerception object.
         * This initializes the new perception node, declares parameters, creates
         * the subscribers and publishers and contains all the necessary variables to
         * set up the algorithm.
         */
        NewPerception();

    private:
        // Subscriber
        std::string kLidarTopic;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;

        /**
         * @brief Callback function for the lidar topic.
         * When the lidar topic recieves a message, this function is called and performs
         * all the necessary steps to generate the suitable point cloud.
         * @param lidar_msg The point cloud message received from the lidar.
         */
        void lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg);
};