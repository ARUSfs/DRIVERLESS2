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
DelaunayTriangulation::DelaunayTriangulation(pcl::PointCloud<ConeXYZColorScore> input_cloud)
{
  pcl_to_vector2(input_cloud);
  triangulation_.insertVertices(cones_vector2_);
}

/**
 * @brief Get the triangulation object Fade_2D type.
 */
CDT::Triangulation<double> DelaunayTriangulation::get_triangulation(){
  return triangulation_;
}

/**
 * @brief Publish the triangulation as a custom message Triangulation to the triangulation topic.
 * 
 * @param triangulation_pub Publisher for the triangulation topic.
 */
void DelaunayTriangulation::publish_triangulation(rclcpp::Publisher<common_msgs::msg::Triangulation>::SharedPtr triangulation_pub){
  common_msgs::msg::Triangulation triangulation_msg;
  std::cout << "1"<< std::endl;
  CDT::TriangleVec triangles_;
  std::cout << "2"<< std::endl;
  triangles_ = triangulation_.triangles;
  std::cout << "3"<< std::endl;
  for (int i=0; i<triangles_.size();i++){
    common_msgs::msg::Simplex triangle;
    common_msgs::msg::PointXY a, b, c;
    CDT::Triangle triangle_;
    triangle_ = triangles_[i];
    CDT::VerticesArr3 vertices_index_ = triangle_.vertices;
    CDT::Triangulation<double>::V2dVec vertices_;
    vertices_ = triangulation_.vertices;
    a.x = vertices_[vertices_index_[0]].x;
    a.y = vertices_[vertices_index_[0]].y;
    b.x = vertices_[vertices_index_[1]].x;
    b.y = vertices_[vertices_index_[1]].y;
    c.x = vertices_[vertices_index_[2]].x;
    c.y = vertices_[vertices_index_[2]].y;
    triangle.points.push_back(a);
    triangle.points.push_back(b);
    triangle.points.push_back(c);
    // triangle.(b);
    // triangle.(c);
    triangulation_msg.simplices.push_back(triangle);
  }
  std::cout << "4"<< std::endl;
  triangulation_msg.header.stamp = clock_->now();
  triangulation_pub->publish(triangulation_msg);

  // std::vector<common_msgs::msg::Simplex> triangles_msg;
  // std::vector<GEOM_FADE2D::Triangle2*> triangles
  // triangulation_.getTrianglePointers(triangles);
  // for (int i = 0; i < triangles.size(); i++){
  //   GEOM_FADE2D::Point2* p, q, r;
  //   triangles[i] -> getCorners(p, q, r);
  //   common_msgs::msg::Simplex triangle;
  //   triangle.a = common_msgs::msg::PointXY(p->x(), p->y());
  //   triangle.b = common_msgs::msg::PointXY(q->x(), q->y());
  //   triangle.c = common_msgs::msg::PointXY(r->x(), r->y());
  //   triangles_msg.push_back(triangle);
  // }
  // triangulation_msg.triangles = triangles_msg;
  // triangulation_pub->publish(triangulation_msg);
}

/**
 * @brief Convert the point cloud library msg to a vector of Point2 objects.
 * 
 * @param input_perception sensor_msgs::msg::PointCloud2::SharedPtr object from the perception.
 */
void DelaunayTriangulation::pcl_to_vector2(pcl::PointCloud<ConeXYZColorScore> input_cloud){
  for (int i = 0; i < input_cloud.size(); i++){
    CDT::V2d<double> point_;
    point_.x = input_cloud[i].x;
    point_.y = input_cloud[i].y;
    cones_vector2_.push_back(point_);
  }
}


