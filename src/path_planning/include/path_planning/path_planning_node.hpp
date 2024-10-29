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
#include <sensor_msgs/msg/point_cloud2.hpp>
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
        rclcpp::Publisher<common_msgs::msg::Simplex>::SharedPtr midpoints_pub_;
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

};