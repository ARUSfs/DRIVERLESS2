/**
 * @file visualization_node.hpp
 * @brief Visualization node header for ARUS Team Driverless pipeline
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <common_msgs/msg/triangulation.hpp>
#include <common_msgs/msg/simplex.hpp>
#include <common_msgs/msg/point_xy.hpp>

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
    private:
        /**
         * @brief Subscriber to the triangulation topic.
         * Callbacks the triangulation visualization.
         */
        rclcpp::Subscription<common_msgs::msg::Triangulation>::SharedPtr triangulation_sub_;

        /**
         * @brief Publisher to the triangulation visualization topic. 
         * Publishes the triangulation visualization.
         */
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr triangulation_visualization_pub_;

        /**
         * @brief Callback for the triangulation topic.
         * Receives the triangulation and publishes the visualization.
         * @param msg common_msgs::msg::Triangulation::SharedPtr Triangulation message.
         */
        void triangulation_callback(const common_msgs::msg::Triangulation::SharedPtr msg);

        /**
         * @brief Topic name for the triangulation topic.
         */
        std::string kTriangulationTopic;

        /**
         * @brief Topic name for the triangulation visualization topic.
         */
        std::string kTriangulationVisualizationTopic;

        rclcpp::Clock::SharedPtr clock_;
};