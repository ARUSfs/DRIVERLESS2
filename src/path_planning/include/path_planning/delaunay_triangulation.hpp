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
#include <path_planning/geom_lib/Fade_2D.h>
#include <path_planning/geom_lib/Point2.h>
#include <path_planning/geom_lib/Triangle2.h>
#include "PointXYZColorScore.h"
#include "common_msgs/msg/point_x_y.hpp"
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
 * triangulation on it. It uses the Fade2D library to perform the triangulation. This class
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
    DelaunayTriangulation(sensor_msgs::msg::PointCloud2::SharedPtr input_perception);

    /**
     * @brief Get the triangulation object
     * 
     * @return Fade_2D The triangulation object
     */
    GEOM_FADE2D::Fade_2D get_triangulation();
  
    /**
     * @brief Publish the triangulation to the triangulation topic as a triangulation msg. 
     * 
     * @param visualization_pub Publisher for the triangulation topic.
     */
    void publish_triangulation(rclcpp::Publisher<common_msgs::msg::Triangulation>::SharedPtr triangulation_pub);


  private:
    GEOM_FADE2D::Fade_2D triangulation_ = GEOM_FADE2D::Fade_2D();
    std::vector<GEOM_FADE2D::Point2> cones_vector2_;
    visualization_msgs::msg::MarkerArray triangulation_array_;
    rclcpp::Clock::SharedPtr clock_;
    std::vector<GEOM_FADE2D::Triangle2*> triangles_;

    /**
     * @brief Convert the point cloud library msg to a vector of Point2 objects
     * 
     * @param input_perception The point cloud from the perception topic
     */
    void pcl_to_vector2(sensor_msgs::msg::PointCloud2::SharedPtr input_perception);
    

};
