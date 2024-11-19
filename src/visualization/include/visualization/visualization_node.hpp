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
        //Subscribers
        rclcpp::Subscription<common_msgs::msg::Triangulation>::SharedPtr triangulation_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr optimized_trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr arussim_trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr delaunay_trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr acc_trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr skidpad_trajectory_sub_;


        //Publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr triangulation_visualization_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_trajectory_visualization_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arussim_trajectory_visualization_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_visualization_pub_;

        //Callbacks
        void triangulation_callback(const common_msgs::msg::Triangulation::SharedPtr msg);
        void optimized_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
        void arussim_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
        void delaunay_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
        void acc_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
        void skidpad_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);

        /**
         * @brief Creates markers for trajectory messages.
         * @param msg common_msgs::msg::Trajectory::SharedPtr Trajectory message.
         * @return visualization_msgs::msg::Marker Marker message.
         */
        visualization_msgs::msg::Marker create_trajectory_marker(
            const common_msgs::msg::Trajectory::SharedPtr msg, bool global=true,
            double red=1.0, double green=0.0, double blue=0.0, double alpha=1.0);

        double kAlpha;

        // Topics to subscribe
        std::string kTriangulationTopic;
        std::string kOptimizedTrajectoryTopic;
        std::string kARUSSimTrajectoryTopic;
        std::string kDelaunayTrajectoryTopic;
        std::string kAccTrajectoryTopic;
        std::string kSkidpadTrajectoryTopic;

        //Topics to publish
        std::string kTriangulationVisualizationTopic;
        std::string kOptimizedTrajectoryVisualizationTopic;
        std::string kARUSSimTrajectoryVisualizationTopic;
        std::string kTrajectoryVisualizationTopic;

};