/**
 * @file delaunay_triangulation.cpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @brief Functions definitions for the ARUS Team's delaunay triangulation in path planning node.
 * @version 0.1
 * @date 2024-10-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "path_planning/delaunay_triangulation.hpp"


/**
 * @class DelaunayTriangulation
 * @brief Construct a new Delaunay Triangulation:: Delaunay Triangulation object to generate a new
 * triangulation Fade_2D object. 
 * 
 * @param input_perception sensor_msgs::msg::PointCloud2::SharedPtr object containing the point cloud
 * from which we are going to generate the triangulation.
 */
DelaunayTriangulation::DelaunayTriangulation(sensor_msgs::msg::PointCloud2::SharedPtr input_perception)
{
  pcl_to_vector2(input_perception);
  triangulation_.insert(cones_vector2_);
}

/**
 * @brief Get the triangulation object Fade_2D type.
 */
GEOM_FADE2D::Fade_2D get_triangulation(){
  return triangulation_;
}

/**
 * @brief Publish the triangulation as a custom message Triangulation to the triangulation topic.
 * 
 * @param triangulation_pub Publisher for the triangulation topic.
 */
void publish_triangulation(rclcpp::Publisher<common_msgs::msg::Triangulation>::SharedPtr triangulation_pub){
  common_msgs::msg::Triangulation triangulation_msg;
  std::vector<common_msgs::msg::Simplex> triangles_msg;
  std::vector<GEOM_FADE2D::Triangle2*> triangles
  triangulation_.getTrianglePointers(triangles);
  for (int i = 0; i < triangles.size(); i++){
    GEOM_FADE2D::Point2* p, q, r;
    triangles[i] -> getCorners(p, q, r);
    common_msgs::msg::Simplex triangle;
    triangle.a = common_msgs::msg::PointXY(p->x(), p->y());
    triangle.b = common_msgs::msg::PointXY(q->x(), q->y());
    triangle.c = common_msgs::msg::PointXY(r->x(), r->y());
    triangles_msg.push_back(triangle);
  }
  triangulation_msg.triangles = triangles_msg;
  triangulation_pub->publish(triangulation_msg);
}

/**
 * @brief Convert the point cloud library msg to a vector of Point2 objects.
 * 
 * @param input_perception sensor_msgs::msg::PointCloud2::SharedPtr object from the perception.
 */
void pcl_to_vector2(sensor_msgs::msg::PointCloud2::SharedPtr input_perception){
  pcl::PointCloud<PointXYZColorScore> pcl_perception;
  pcl::fromROSMsg(*input_perception, pcl_perception);
  for (int i = 0; i < pcl_perception.size(); i++){
    cones_vector2_.push_back(Point2(pcl_perception[i].x, pcl_perception[i].y));
  }
}


