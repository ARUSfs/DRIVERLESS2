/**
 * @file delaunay_triangulation.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @brief Header file for the ARUS Team's delaunay triangulation in path planning node.
 *  
 * This file contains the declaration of the DelaunayTriangulation class, which is used to 
 * perform Delaunay triangulation on a point cloud. It also includes visualization features.
 * @version 0.1
 * @date 14-10-2024
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "ConeXYZColorScore.h"
#include "CDT.h"
#include <path_planning/Triangulation.h>
#include <path_planning/CDTUtils.h>
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/simplex.hpp"
#include "common_msgs/msg/triangulation.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <iostream>

/**
 * @class DelaunayTriangulation
 * @brief Construct a new Delaunay Triangulation object
 * 
 * This class takes a point cloud from the perception topic as a msg and performs Delaunay 
 * triangulation on it. It uses the CDT library to perform the triangulation. This class
 * also generates the visualization of the triangulation.
 */

class DelaunayTriangulation
{
  public:
    /**
     * @brief Construct a new Delaunay Triangulation object
     * 
     * @param input_perception The point cloud from the perception topic
     */
    DelaunayTriangulation(pcl::PointCloud<ConeXYZColorScore> input_cloud);

    /**
     * @brief Get the triangulation object
     * 
     * @return CDT The triangulation object
     */
    CDT::Triangulation<double> get_triangulation();
  
    /**
     * @brief Publish the triangulation to the triangulation topic as a triangulation msg. 
     * 
     * @param visualization_pub Publisher for the triangulation topic.
     */
    void publish_triangulation(rclcpp::Publisher<common_msgs::msg::Triangulation>::SharedPtr triangulation_pub);


  private:
    CDT::Triangulation<double> triangulation_;
    std::vector<CDT::V2d<double>> cones_vector2_;
    visualization_msgs::msg::MarkerArray triangulation_array_;
    rclcpp::Clock::SharedPtr clock_;
    

    /**
     * @brief Convert the point cloud library msg to a vector of Point2 objects
     * 
     * @param input_perception The point cloud from the perception topic
     */
    void pcl_to_vector2(pcl::PointCloud<ConeXYZColorScore> input_cloud);
    

};
