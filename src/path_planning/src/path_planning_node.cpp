/**
 * @file path_planning_node.cpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @brief Main file for the Path Planning node. Contains the main function and the implementation 
 * of the methods to achieve a robust and reliable path planning algorithm for the ARUS Team 
 * which extracts the midpoints of the track that the ART will follow.
 * @version 0.1
 * @date 25-10-2024
 * 
 */
#include "path_planning/path_planning_node.hpp"

PathPlanning::PathPlanning() : Node("path_planning")
{
    this->declare_parameter<std::string>("perception_topic", "/perception");
    this->declare_parameter<std::string>("triangulation_topic", "/triangulation"); 
    this->get_parameter("perception_topic", kPerceptionTopic);
    this->get_parameter("triangulation_topic", kTriangulationTopic);

    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kPerceptionTopic, 10, std::bind(&PathPlanning::perception_callback, this, std::placeholders::_1));
    triangulation_pub_ = this->create_publisher<common_msgs::msg::Triangulation>(kTriangulationTopic, 10);
    
    

}

/**
 * @brief Create callback function for the perception topic. Main pipeline of the path planning node.
 * 
 * @param msg Point Cloud 2 message from the perception topic.
 */
void PathPlanning::perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr per_msg)
{   
    pcl::PointCloud<ConeXYZColorScore> pcl_cloud_;
    pcl::fromROSMsg(*per_msg, pcl_cloud_);
    CDT::Triangulation<double> triangulation_;
    triangulation_ = PathPlanning::create_triangulation(pcl_cloud_);
    common_msgs::msg::Triangulation triangulation_msg;
    if (triangulation_.isFinalized()){
        triangulation_msg = PathPlanning::create_triangulation_msg(triangulation_);
    }
    triangulation_pub_ -> publish(triangulation_msg);


}

CDT::Triangulation<double> PathPlanning::create_triangulation(pcl::PointCloud<ConeXYZColorScore> input_cloud){
    CDT::Triangulation<double> triangulation;
    std::vector<CDT::V2d<double>> points_;
    for (int i = 0; i<input_cloud.size();i++){
        ConeXYZColorScore cone_ = input_cloud.points[i];
        CDT::V2d<double> point_ = CDT::V2d<double>::make(cone_.x, cone_.y);
        points_.push_back(point_);
    }
    triangulation.insertVertices(points_);
    triangulation.eraseSuperTriangle();
    return triangulation;
}

common_msgs::msg::Triangulation PathPlanning::create_triangulation_msg(CDT::Triangulation<double> triangulation){
    common_msgs::msg::Triangulation triangulation_msg;
    CDT::Triangulation<double>::V2dVec points_ = triangulation.vertices;
    CDT::TriangleVec triangles_ = triangulation.triangles;
    for (int i = 0; i < triangles_.size(); i++){
        common_msgs::msg::Simplex triangle;
        common_msgs::msg::PointXY a, b, c;
        CDT::Triangle triangle_ = triangles_[i];
        CDT::VerticesArr3 vertices_index_ = triangle_.vertices;
        CDT::VertInd a_ind_ = vertices_index_[0];
        CDT::VertInd b_ind_ = vertices_index_[1];
        CDT::VertInd c_ind_ = vertices_index_[2];
        a.x = points_[a_ind_].x;
        a.y = points_[a_ind_].y;
        b.x = points_[b_ind_].x;
        b.y = points_[b_ind_].y;
        c.x = points_[c_ind_].x;
        c.y = points_[c_ind_].y;
        triangle.points = {a, b, c};
        triangulation_msg.simplices.push_back(triangle);
    }
    return triangulation_msg;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}