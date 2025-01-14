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
#include "CDT.h"
#include <Triangulation.h>
#include <CDTUtils.h>
#include "ConeXYZColorScore.h"
#include "path_planning/simplex_tree.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <common_msgs/msg/point_xy.hpp>
#include <common_msgs/msg/simplex.hpp>
#include <common_msgs/msg/triangulation.hpp>
#include <common_msgs/msg/trajectory.hpp>
#include "common_msgs/msg/state.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>


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
        // Parameters from configuration file
        std::string kPerceptionTopic;
        std::string kTriangulationTopic;
        std::string kTrajectoryTopic;
        double kMaxTriLen;
        double kMaxTriAngle;
        double kLenCoeff;
        double kAngleCoeff;
        double kMaxAngle;
        double kMaxVel;
        double kMaxYAcc;
        double kMaxXAcc;
        double kSmooth;
        
        // Suscribers and publishers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;
        rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
        rclcpp::Publisher<common_msgs::msg::Triangulation>::SharedPtr triangulation_pub_;
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

        // CarState
        double x_;
        double y_;
        double yaw_;
        double vx_;
        double vy_;
        double v_;

        // Triangulation attributes
        CDT::TriangleVec triangles_;
        CDT::Triangulation<double>::V2dVec vertices_;
        
        // Routes
        std::vector<std::vector<int>> triangle_routes_;
        std::vector<std::vector<CDT::V2d<double>>> midpoint_routes_;
        std::vector<CDT::V2d<double>> best_midpoint_route_;

        /**
         * @brief Callback function for the perception topic.
         * When the perception topic recieves a message, this function is called and performs
         * all the necessary steps to generate the planning.
         * 
         * @param per_msg The point cloud received from the perception.
         */
        void perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg);

        /**
         * @brief Callback function for the car state topic.
         * Update the car state variables when a new message is received.
         * @param state_msg Common_msgs message containing the car state. Only the x, y and yaw are used.
         */
        void car_state_callback(common_msgs::msg::State::SharedPtr state_msg);
        
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
         * @brief Get the mid points of the edges of the triangulation. 
         * It returns a vector of V2d points containing the mid points without duplicates.
         * @param triangulation CDT object containing the triangulation.
         * @return std::vector<CDT::V2d<double>> 
         */
        std::vector<CDT::V2d<double>> get_midpoints(CDT::Triangulation<double> triangulation);

        /**
         * @brief Calculate the euclidean norm of a vector. 
         * i.e.: if v=(x,y), norm(v) = (x^2 + y^2)^(1/2).
         * @param v Vector to calculate the norm.
         * @return double 
         */
        double norm(CDT::V2d<double> v);

        /**
         * @brief Get the closest midpoint to the origin as a CDT 2D vector.
         * 
         * @param midpoint_arr Array of midpoints to calculate the closest one.
         * @return CDT::V2d<double> 
         */
        CDT::V2d<double> get_closest_midpoint(std::vector<CDT::V2d<double>> midpoint_arr);

        /**
         * @brief Get the closest triangle object using the centroid as reference.
         * @return int Index of the closest triangle found.
         */
        int get_closest_triangle();

        /**
         * @brief Calulate the centroid of a triangle given its index in the triangulation.
         * Centroid is defined as the average of the vertices of the triangle.
         * @param triangle_ind int index of the triangle in the triangulation.
         * @return CDT::V2d<double> point containing the centroid of the triangle.
         */
        CDT::V2d<double> compute_centroid(int triangle_ind);

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
         * @brief Get the edge shared by two neighbor triangles.
         * @param triangle1 CDT::Triangle first triangle.
         * @param triangle2 CDT::Triangle second triangle.
         * @return CDT::Edge edge shared by the two triangles.
         */
        CDT::Edge get_share_edge(CDT::Triangle triangle1, CDT::Triangle triangle2);

        /**
         * @brief Transform the triangle routes into a vector of the midpoints crossed by the route.
         */
        void get_midpoint_routes();

        /**
         * @brief Calculate the cost of a given route based on the distance and angle between consecutive points and
         * the total distance of the route. If the route has a "stop condition" (i.e. the angle between two segments is
         * bigger than a threshold), the route is modified to stop before the stop condition.
         * @param route vector of CDT::V2d<double> points containing the route through the midpoints.
         * @return float result of the cost function.
         */
        double get_route_cost(std::vector<CDT::V2d<double>> &route);

        

        /**
         * @brief Create a trajectory msg object from a given route.
         * @param route std::vector<CDT::V2d<double>> vector containing the route.
         * @return common_msgs::msg::Trajectory parsed trajectory message to ROS2 format.
         */
        common_msgs::msg::Trajectory create_trajectory_msg(std::vector<CDT::V2d<double>> route);
};