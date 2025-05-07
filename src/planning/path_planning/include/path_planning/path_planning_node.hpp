/**
 * @file path_planning_node.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @brief Header file for the Path Planning node. Contains the class definition and the declaration
 * of the methods used in the algorithm.
 */

// General libraries
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

// Triangulation libraries
#include <CDT.h>
#include <Triangulation.h>
#include <CDTUtils.h>

// ROS2 libraries and msg tipes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int16.hpp>
#include <common_msgs/msg/point_xy.hpp>
#include <common_msgs/msg/simplex.hpp>
#include <common_msgs/msg/triangulation.hpp>
#include <common_msgs/msg/trajectory.hpp>
#include <common_msgs/msg/car_info.hpp>
#include <common_msgs/msg/track_limits.hpp>
#include "ConeXYZColorScore.h"

// Custom libraries
#include "simplex_tree.hpp"
#include "utils.hpp"


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
         */
        PathPlanning();
    private:
        // Variables
        // CarState
        double x_=0;
        double y_=0;
        double yaw_;
        double target_speed_ = 0;
        double vx_;
        double vy_;
        double v_;
        ConeXYZColorScore origin_ = ConeXYZColorScore();

        // Lap count and state of the algorithm
        int lap_count_ = 0;
        bool track_limits_sent_ = false;
        rclcpp::Time last_unfinished_;

        // Point cloud
        pcl::PointCloud<ConeXYZColorScore> pcl_cloud_;

        // Triangulation attributes
        CDT::TriangleVec triangles_;
        CDT::Triangulation<double>::V2dVec vertices_;
        
        // Routes
        std::vector<std::vector<ConeXYZColorScore>> midpoint_routes_;
        std::vector<ConeXYZColorScore> best_midpoint_route_;
        std::vector<ConeXYZColorScore> closing_route_= {};
        std::vector<std::vector<ConeXYZColorScore>> previous_midpoint_routes_;
        int invalid_counter_=0;
        std::vector<ConeXYZColorScore> back_route_ = {ConeXYZColorScore(0,0,0,UNCOLORED,-1)};
        std::vector<ConeXYZColorScore> back_points_;
        CDT::TriangleVec TL_triang_; 
        std::vector<int> TL_tri_indices_;

        // Track limits
        std::vector<ConeXYZColorScore> left_limit_;
        std::vector<ConeXYZColorScore> right_limit_;

        // Parameters
        // Debug
        bool kDebug;
        // Topics
        std::string kMapTopic;
        std::string kLapCountTopic;
        std::string kCarInfoTopic;
        std::string kOptimizerTopic;
        std::string kTriangulationTopic;
        std::string kTrajectoryTopic;
        std::string kTrackLimitsTopic;

        // Triangulation 
        double kMaxTriLen;

        // Route
        double kAngleCoeff;
        double kLenCoeff;
        double kSmoothCoeff;
        double kPrevRouteBias;
        int kRouteBack;
        bool kUseBuffer;
        double kTimeToClose;
        bool kStopAfterClosing;

        // Profile creation
        double kMaxVel;
        double kMaxYAcc;
        double kMaxXAcc;

        // Suscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lap_count_sub_;
        rclcpp::Subscription<common_msgs::msg::CarInfo>::SharedPtr car_info_sub_;
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr optimizer_sub_;

        // Publishers
        rclcpp::Publisher<common_msgs::msg::Triangulation>::SharedPtr triangulation_pub_;
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
        rclcpp::Publisher<common_msgs::msg::TrackLimits>::SharedPtr track_limits_pub_;


        /**
         * @brief Callback function for the perception topic.
         */
        void map_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg);

        /**
         * @brief Callback function for the car info topic.
         * Update the car state variables when a new message is received.
         */
        void car_info_callback(common_msgs::msg::CarInfo::SharedPtr state_msg);

        /**
         * @brief Callback funtion to actualize lap counter.
         */
        void lap_count_callback(std_msgs::msg::Int16::SharedPtr lap_msg);
        
        /**
         * @brief Callback function for the optimizer topic.
         * Shutdown the node when a message is received from the optimizer.
         */
        void optimizer_callback(common_msgs::msg::Trajectory::SharedPtr optimizer_msg);

        /**
         * @brief Create a triangulation object from the point cloud and delete triangles with long 
         * edges.
         */
        CDT::Triangulation<double> create_triangulation(pcl::PointCloud<ConeXYZColorScore> input_cloud);

        /**
         * @brief Get the back edge given the position of the car.  
         */
        std::vector<ConeXYZColorScore> get_back_edge();

        /**
         * @brief Get the final route after comparing with previous routes to avoid outliers.
         */
        std::vector<ConeXYZColorScore> get_final_route();

        /**
         * @brief Create a trajectory msg object from a given route, adding the speed and acceleration
         * profiles. Smooth the trajectory if required.
         */
        common_msgs::msg::Trajectory create_trajectory_msg(std::vector<ConeXYZColorScore> route, bool smooth = true);

        /**
         * @brief Create a track limits msg object from a given route of triangles and the triangulation.
         */
        common_msgs::msg::TrackLimits create_track_limits_msg(CDT::TriangleVec triang, 
                                                              std::vector<int> triangles_route);
        
        /**
         * @brief Add the back edge to the track limits if each cone is colored and not too close to
         * the last cone of the limit (avoid duplicates).
         */
        void add_to_track_limits(std::vector<ConeXYZColorScore> back_edge);

        /**
         * @brief Create a triangulation msg object from the triangulation
         */
        common_msgs::msg::Triangulation create_triangulation_msg(CDT::Triangulation<double> triangulation);
};