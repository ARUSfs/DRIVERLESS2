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
#include <common_msgs/msg/track_limits.hpp>
#include <std_msgs/msg/float32.hpp>

/**
 * @class Visualization
 * @brief Visualization class 
 * 
 * This class represents the pipeline topics through marker messages in rviz2
 */
class Visualization : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the Visualization class.
         */
        Visualization();
    private:
        // Topics to subscribe
        std::string kTriangulationTopic;
        std::string kOptimizedTrajectoryTopic;
        std::string kARUSSimTrajectoryTopic;
        std::string kDelaunayTrajectoryTopic;
        std::string kAccTrajectoryTopic;
        std::string kSkidpadTrajectoryTopic;
        std::string kPursuitPointTopic;
        std::string kTrackLimitsTopic;

        //Topics to publish
        std::string kTriangulationVisualizationTopic;
        std::string kOptimizedTrajectoryVisualizationTopic;
        std::string kARUSSimTrajectoryVisualizationTopic;
        std::string kTrajectoryVisualizationTopic;
        std::string kPursuitPointVisualizationTopic;
        std::string kTrackLimitsVisualizationTopic;
        std::string kOptimizedTrajectory3DVisualizationTopic;

        // Visualization parameters
        double kAlpha;
    
        //Subscribers
        rclcpp::Subscription<common_msgs::msg::Triangulation>::SharedPtr triangulation_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr optimized_trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr arussim_trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr delaunay_trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr acc_trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr skidpad_trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::PointXY>::SharedPtr pursuit_point_sub_;
        rclcpp::Subscription<common_msgs::msg::TrackLimits>::SharedPtr track_limits_sub_;

        //Publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr triangulation_visualization_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_trajectory_visualization_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_colored_trajectory_visualization_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arussim_trajectory_visualization_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_visualization_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pursuit_point_visualization_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_limits_vis_pub_;

        //Callbacks
        void triangulation_callback(const common_msgs::msg::Triangulation::SharedPtr msg);
        void optimized_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
        void arussim_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
        void delaunay_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
        void acc_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
        void skidpad_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);
        void pursuit_point_callback(const common_msgs::msg::PointXY::SharedPtr msg);
        void track_limits_callback(const common_msgs::msg::TrackLimits::SharedPtr msg);

        /**
         * @brief Creates markers for trajectory messages.
         * @param msg common_msgs::msg::Trajectory::SharedPtr Trajectory message.
         * @return visualization_msgs::msg::Marker Marker message.
         */
        visualization_msgs::msg::Marker create_trajectory_marker(
            const common_msgs::msg::Trajectory::SharedPtr msg, bool global=true,
            double red=1.0, double green=0.0, double blue=0.0, double alpha=1.0);


        /**
         * @brief Creates a 3D trajectory marker.
         * @param msg Trajectory message.
         * @param global If true, sets the frame_id to "arussim/world", otherwise "arussim/vehicle_cog".
         * @return visualization_msgs::msg::Marker 3D trajectory marker.
         */
        visualization_msgs::msg::Marker create_trajectory_3D_marker(
            const common_msgs::msg::Trajectory::SharedPtr msg, bool global=true,
            double red=1.0, double green=0.0, double blue=0.0, double alpha=1.0);

};