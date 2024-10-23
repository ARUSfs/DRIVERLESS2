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
    kTriangulationVisualizationTopic = "/visualization/triangulation";

    triangulation_sub_ = this->create_subscription<common_msgs::msg::Triangulation>(
        kTriangulationTopic, 10, std::bind(&Visualization::triangulation_callback, this, std::placeholders::_1));

    triangulation_visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        kTriangulationVisualizationTopic, 10);

    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
}

void Visualization::triangulation_callback(const common_msgs::msg::Triangulation::SharedPtr msg)
{
    visualization_msgs::msg::MarkerArray marker_array_;
    for (int i = 0; i < msg -> simplices.size(); i++){
        visualization_msgs::msg::Marker marker_;
        marker_.header.frame_id = "arussim/vehicle_cog";
        marker_.header.stamp = clock_->now();
        marker_.ns = "visualization";
        marker_.id = i;
        marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.pose.position.x = 0;
        marker_.pose.position.y = 0;
        marker_.pose.position.z = 0;
        marker_.scale.x = 0.1;
        marker_.scale.y = 0.1;
        marker_.scale.z = 0.1;
        marker_.color.a = 1.0;
        marker_.color.r = 1.0;
        marker_.color.g = 0.0;
        marker_.color.b = 0.0;
        marker_.lifetime = rclcpp::Duration::from_seconds(0.0);
        for (int j=0; j<3; j++){
            geometry_msgs::msg::Point p;
            p.x = msg->simplices[i].points[j].x;
            p.y = msg->simplices[i].points[j].y;
            p.z = 0;
            marker_.points.push_back(p);
        }
        geometry_msgs::msg::Point p;
        p.x = msg->simplices[i].points[0].x;
        p.y = msg->simplices[i].points[0].y;
        p.z = 0;
        marker_.points.push_back(p);
        marker_array_.markers.push_back(marker_);
    }
    triangulation_visualization_pub_->publish(marker_array_);
    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualization>());
    rclcpp::shutdown();
    return 0;
}