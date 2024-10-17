#include "path_planning/path_planning_node.hpp"
#include <path_planning/delaunay_triangulation.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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
void PathPlanning::perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{



}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}