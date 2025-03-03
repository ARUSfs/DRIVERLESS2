/**
 * @file path_planning_node.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @brief Header file for the Path Planning node. 
 * Contains the class definition and the declaration of the methods 
 * used in the algorithm.
 * @version 0.1
 * @date 23-10-2024
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
         * 
         * This initializes the Path Planning node, declares parameters, creates
         * the subscribers and publishers and contains all the necessary variables to
         * set up the algorithm.
         */
        PathPlanning();
    private:
        // Topics
        std::string kMapTopic;
        std::string kLapCountTopic;
        std::string kCarInfoTopic;
        std::string kOptimizerTopic;
        std::string kTriangulationTopic;
        std::string kTrajectoryTopic;
        std::string kTrackLimitsTopic;

        // Triangulation parameters
        double kMaxTriLen;
        double kMaxTriAngle;
        bool kColor;

        // Route parameters
        double kAngleCoeff;
        double kLenCoeff;
        double kSmoothCoeff;
        double kPrevRouteBias;
        int kRouteBack;
        bool kUseBuffer;
        bool kUseClosingRoute;
        bool kStopAfterClosing;

        // Profile creation parameters
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

        // Point cloud
        pcl::PointCloud<ConeXYZColorScore> pcl_cloud_;

        // Triangulation attributes
        CDT::TriangleVec triangles_;
        CDT::Triangulation<double>::V2dVec vertices_;
        
        // Routes
        std::vector<std::vector<ConeXYZColorScore>> midpoint_routes_;
        std::vector<ConeXYZColorScore> best_midpoint_route_;
        std::vector<int> best_index_route_;
        std::vector<ConeXYZColorScore> closing_route_= {};
        std::vector<std::vector<ConeXYZColorScore>> previous_midpoint_routes_;
        int invalid_counter_=0;

        /**
         * @brief Callback function for the perception topic.
         * When the perception topic recieves a message, this function is called and performs
         * all the necessary steps to generate the planning.
         * 
         * @param per_msg The point cloud received from the perception.
         */
        void map_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg);

        /**
         * @brief Callback funtion to actualize lap counter.
         * @param lap_msg int16 message containing the lap count.
         */
        void lap_count_callback(std_msgs::msg::Int16::SharedPtr lap_msg);

        /**
         * @brief Callback function for the car info topic.
         * Update the car state variables when a new message is received.
         * @param state_msg Common_msgs message containing the car state.
         */
        void car_info_callback(common_msgs::msg::CarInfo::SharedPtr state_msg);
        
        /**
         * @brief Callback function for the optimizer topic.
         * Shutdown the node when a message is received from the optimizer.
         * @param optimizer_msg Common_msgs message containing the optimized trajectory.
         */
        void optimizer_callback(common_msgs::msg::Trajectory::SharedPtr optimizer_msg);

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

        /**
         * @brief Calculate the euclidean norm of a vector. 
         * i.e.: if v=(x,y), norm(v) = (x^2 + y^2)^(1/2).
         * @param v Vector to calculate the norm.
         * @return double 
         */
        double norm(CDT::V2d<double> v);

        /**
         * @brief Get the index of the origin vertex in the triangulation.
         * @return int index of the origin vertex.
         */
        int get_vertex_index(CDT::V2d<double> vertex);

        /**
         * @brief Get the triangles adjacent to a vertex from its index.
         * @param vert_index int index of the vertex.
         * @return std::vector<int> vector of index of the triangles adjacent to the vertex.
         */
        std::vector<int> get_triangles_from_vert(int vert_index);

        /**
         * @brief Get the final route after comparing with previous routes to avoid outliers.
         * Compare the best route in last iteration to the n (6) previous and count in how many of 
         * them the route is present in more than x% (75%) of the points. In that case, return the
         * route, otherwise return the second to last route.
         * @return std::vector<CDT::V2d<double>> final route to be published.
         */
        std::vector<ConeXYZColorScore> get_final_route();

        /**
         * @brief Create a trajectory msg object from a given route.
         * @param route std::vector<CDT::V2d<double>> vector containing the route.
         * @return common_msgs::msg::Trajectory parsed trajectory message to ROS2 format.
         */
        common_msgs::msg::Trajectory create_trajectory_msg(std::vector<ConeXYZColorScore> route);

        /**
         * @brief Create a track limits msg object from a given route.
         * 
         * @param triangle_route std::vector<int> vector containing the triangles of the route. 
         * This route is suposed to be the final and closed route once the first lap is completed.
         * @return common_msgs::msg::TrackLimits the message to be published.
         */
        common_msgs::msg::TrackLimits create_track_limits_msg(std::vector<int> triangle_route);
};