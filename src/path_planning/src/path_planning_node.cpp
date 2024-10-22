#include "path_planning/path_planning_node.hpp"
// #include <path_planning/delaunay_triangulation.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
    CDT::Triangulation<double> triangulation_;
    std::vector<CDT::V2d<double>> points_;
    pcl::fromROSMsg(*per_msg, pcl_cloud_);
    // RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", pcl_cloud_.size());
    for (int i = 0; i<pcl_cloud_.size();i++){
        ConeXYZColorScore cone_ = pcl_cloud_.points[i];
        std::cout << cone_.x << " " << cone_.y << std::endl;
        CDT::V2d<double> point_ = CDT::V2d<double>::make(cone_.x, cone_.y);
        points_.push_back(point_);
    }
    triangulation_.insertVertices(points_);
    triangulation_.eraseSuperTriangle();
    std::cout << "Triangulation done" << std::endl;
    common_msgs::msg::Triangulation triangulation_msg;
    // CDT::Triangulation<double>::V2dVec vertices_ = triangulation_.vertices;
    if (triangulation_.isFinalized()){ // Si se va a usar el isFinalized hay que quitar el outer triangle
        CDT::TriangleVec triangles_ = triangulation_.triangles;
        for (int i = 0; i < triangles_.size(); i++){
            // std::cout << "Triangle " << i << std::endl;
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
    }
    triangulation_pub_ -> publish(triangulation_msg);

    // DelaunayTriangulation del(pcl_cloud_);    
    // del.publish_triangulation(triangulation_pub_);
    // DelaunayTriangulation del_ = DelaunayTriangulation(*per_msg);
    // // triangulation_geom_ = triangulation_class_.get_triangulation();
    // sensor_msgs::msg::PointCloud2 msg_out = *msg;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}