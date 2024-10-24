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
    kTriangulationTopic = "/path_planning/triangulation";
    kARUSSimTrajectoryTopic = "/arussim_interface/fixed_trajectory";
    kTriangulationVisualizationTopic = "/visualization/triangulation";
    kARUSSimTrajectoryVisualizationTopic = "/visualization/arussim_fixed_trajectory";

    clock_ = this->get_clock();

    triangulation_visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        kTriangulationVisualizationTopic, 10);
    arussim_trajectory_visualization_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        kARUSSimTrajectoryVisualizationTopic, 10);

    triangulation_sub_ = this->create_subscription<common_msgs::msg::Triangulation>(
        kTriangulationTopic, 10, std::bind(&Visualization::triangulation_callback, this, std::placeholders::_1));
    arussim_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kARUSSimTrajectoryTopic, 10, std::bind(&Visualization::arussim_trajectory_callback, this, std::placeholders::_1));


}

void Visualization::triangulation_callback(const common_msgs::msg::Triangulation::SharedPtr msg)
{
    visualization_msgs::msg::MarkerArray marker_array;
    for (int i = 0; i < msg -> simplices.size(); i++){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "arussim/vehicle_cog";
        marker.header.stamp = clock_->now();
        marker.ns = "visualization";
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
        marker.color.r = 1.0;
        marker.color.g = 0.0;
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
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "arussim/world";
    marker.header.stamp = clock_->now();
    marker.ns = "visualization";
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
    
    arussim_trajectory_visualization_pub_->publish(marker);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualization>());
    rclcpp::shutdown();
    return 0;
}