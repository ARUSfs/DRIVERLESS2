/**
 * @file visualization_node.hpp
 * @brief Visualization node header for ARUS Team Driverless pipeline
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <common_msgs/msg/trajectory.hpp>
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
         */
        rclcpp::Subscription<common_msgs::msg::Triangulation>::SharedPtr triangulation_sub_;
        /**
         * @brief Subscriber to the trajectory topic.
         */
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr arussim_trajectory_sub_;

        /**
         * @brief Publisher to the triangulation visualization topic. 
         */
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr triangulation_visualization_pub_;
        /**
         * @brief Publisher to the trajectory visualization topic. 
         */
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arussim_trajectory_visualization_pub_;

        /**
         * @brief Callback for the triangulation topic.
         * @param msg common_msgs::msg::Triangulation::SharedPtr Triangulation message.
         */
        void triangulation_callback(const common_msgs::msg::Triangulation::SharedPtr msg);
        /**
         * @brief Callback for trajectory topics.
         * @param msg common_msgs::msg::Trajectory::SharedPtr Trajectory message.
         */
        void arussim_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);

        /**
         * @brief Creates markers for trajectory messages.
         * @param msg common_msgs::msg::Trajectory::SharedPtr Trajectory message.
         * @return visualization_msgs::msg::Marker Marker message.
         */
        visualization_msgs::msg::Marker create_trajectory_marker(
            const common_msgs::msg::Trajectory::SharedPtr msg);

        /**
         * @brief Topic name for the triangulation topic.
         */
        std::string kTriangulationTopic;
        std::string kARUSSimTrajectoryTopic;

        /**
         * @brief Topic name for the triangulation visualization topic.
         */
        std::string kTriangulationVisualizationTopic;
        std::string kARUSSimTrajectoryVisualizationTopic;

        rclcpp::Clock::SharedPtr clock_;
};