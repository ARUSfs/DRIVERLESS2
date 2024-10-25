/**
 * @file visualization_node.cpp
 * @brief Visualization node implementaion for ARUS Team Driverless pipeline
 */

#include "visualization/visualization_node.hpp"


/**
 * @class Visualization
 * @brief Visualization class 
 * 
 * This class represents the pipeline topics through marker messages 
 * in rviz2
 */
Visualization::Visualization() : Node("visualization")
{   
    this->declare_parameter("triangulation_topic", "/path_planning/triangulation");
    this->declare_parameter("optimized_trajectory_topic", "/trajectory_optimizer/trajectory");
    this->declare_parameter("arussim_trajectory_topic", "/arussim_interface/fixed_trajectory");
    this->declare_parameter("triangulation_visualization_topic", "/visualization/triangulation");
    this->declare_parameter("optimized_trajectory_visualization_topic", "/visualization/optimized_trajectory");
    this->declare_parameter("arussim_trajectory_visualization_topic", "/visualization/arussim_fixed_trajectory");
    
    this->get_parameter("triangulation_topic", kTriangulationTopic);
    this->get_parameter("optimized_trajectory_topic", kOptimizedTrajectoryTopic);
    this->get_parameter("arussim_trajectory_topic", kARUSSimTrajectoryTopic);
    this->get_parameter("triangulation_visualization_topic", kTriangulationVisualizationTopic);
    this->get_parameter("optimized_trajectory_visualization_topic", kOptimizedTrajectoryVisualizationTopic);
    this->get_parameter("arussim_trajectory_visualization_topic", kARUSSimTrajectoryVisualizationTopic);

    triangulation_visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        kTriangulationVisualizationTopic, 10);
    optimized_trajectory_visualization_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        kOptimizedTrajectoryVisualizationTopic, 10);
    arussim_trajectory_visualization_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        kARUSSimTrajectoryVisualizationTopic, 10);

    triangulation_sub_ = this->create_subscription<common_msgs::msg::Triangulation>(
        kTriangulationTopic, 10, std::bind(&Visualization::triangulation_callback, this, std::placeholders::_1));
    optimized_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kOptimizedTrajectoryTopic, 10, std::bind(&Visualization::optimized_trajectory_callback, this, std::placeholders::_1));
    arussim_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kARUSSimTrajectoryTopic, 10, std::bind(&Visualization::arussim_trajectory_callback, this, std::placeholders::_1));


}

void Visualization::triangulation_callback(const common_msgs::msg::Triangulation::SharedPtr msg)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    for (int i = 0; i < msg -> simplices.size(); i++){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "arussim/vehicle_cog";
        marker.ns = "triangulation";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        for (int j=0; j<3; j++){
            geometry_msgs::msg::Point p;
            p.x = msg->simplices[i].points[j].x;
            p.y = msg->simplices[i].points[j].y;
            p.z = 0;
            marker.points.push_back(p);
        }
        geometry_msgs::msg::Point p;
        p.x = msg->simplices[i].points[0].x;
        p.y = msg->simplices[i].points[0].y;
        p.z = 0;
        marker.points.push_back(p);
        marker_array.markers.push_back(marker);
    }
    triangulation_visualization_pub_->publish(marker_array);
    
}

void Visualization::arussim_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker = this->create_trajectory_marker(msg);
    arussim_trajectory_visualization_pub_->publish(marker);
}

void Visualization::optimized_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker = this->create_trajectory_marker(msg);
    optimized_trajectory_visualization_pub_->publish(marker);
}

visualization_msgs::msg::Marker Visualization::create_trajectory_marker(
    const common_msgs::msg::Trajectory::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "arussim/world";
    marker.ns = "trajectory";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    for (int i=0; i<msg->points.size(); i++){
        geometry_msgs::msg::Point p;
        p.x = msg->points[i].x;
        p.y = msg->points[i].y;
        p.z = 0;
        marker.points.push_back(p);
    }
    return marker;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualization>());
    rclcpp::shutdown();
    return 0;
}