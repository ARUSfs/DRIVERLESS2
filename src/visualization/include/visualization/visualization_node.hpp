/**
 * @file visualization_node.hpp
 * @brief Visualization node header for ARUS Team Driverless pipeline
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>


/**
 * @class Visualization
 * @brief Visualization class 
 * 
 * This class represents the pipeline topics through marker messages 
 * in rviz2
 */
class Visualization : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the Visualization class.
         */
        Visualization();

};