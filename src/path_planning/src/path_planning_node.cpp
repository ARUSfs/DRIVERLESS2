#include "path_planning/path_planning_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

PathPlanningNode::PathPlanningNode() : Node("path_planning")
{
    this->declare_parameter<std::string>("perception_topic", "/perception");


    this->get_parameter("perception_topic", kPerceptionTopic);

    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kPerceptionTopic, 10, std::bind(&PathPlanningNode::perception_callback, this, std::placeholders::_1));
}

void PathPlanningNode::perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanningNode>());
    rclcpp::shutdown();
    return 0;
}