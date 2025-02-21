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
    this->declare_parameter("alpha", 1.0);
    this->declare_parameter("triangulation_topic", "/path_planning/triangulation");
    this->declare_parameter("optimized_trajectory_topic", "/trajectory_optimization/trajectory");
    this->declare_parameter("arussim_trajectory_topic", "/arussim_interface/fixed_trajectory");
    this->declare_parameter("delaunay_trajectory_topic", "/path_planning/trajectory");
    this->declare_parameter("acc_trajectory_topic", "/acc_planning/trajectory");
    this->declare_parameter("skidpad_trajectory_topic", "/skidpad_planning/trajectory");
    this->declare_parameter("triangulation_visualization_topic", "/visualization/triangulation");
    this->declare_parameter("optimized_trajectory_visualization_topic", "/visualization/optimized_trajectory");
    this->declare_parameter("arussim_trajectory_visualization_topic", "/visualization/arussim_fixed_trajectory");
    this->declare_parameter("trajectory_visualization_topic", "/visualization/trajectory");
    this->declare_parameter("pursuit_point_topic", "/controller/pursuit_point");
    this->declare_parameter("pursuit_point_visualization_topic", "/visualization/pursuit_point");
    this->declare_parameter("track_limits_topic", "/path_planning/track_limits");
    this->declare_parameter("track_limits_visualization_topic", "/visualization/track_limits");

    this->get_parameter("alpha", kAlpha);
    this->get_parameter("triangulation_topic", kTriangulationTopic);
    this->get_parameter("optimized_trajectory_topic", kOptimizedTrajectoryTopic);
    this->get_parameter("arussim_trajectory_topic", kARUSSimTrajectoryTopic);
    this->get_parameter("delaunay_trajectory_topic", kDelaunayTrajectoryTopic);
    this->get_parameter("acc_trajectory_topic", kAccTrajectoryTopic);
    this->get_parameter("skidpad_trajectory_topic", kSkidpadTrajectoryTopic);
    this->get_parameter("triangulation_visualization_topic", kTriangulationVisualizationTopic);
    this->get_parameter("optimized_trajectory_visualization_topic", kOptimizedTrajectoryVisualizationTopic);
    this->get_parameter("arussim_trajectory_visualization_topic", kARUSSimTrajectoryVisualizationTopic);
    this->get_parameter("trajectory_visualization_topic", kTrajectoryVisualizationTopic);
    this->get_parameter("pursuit_point_topic", kPursuitPointTopic);
    this->get_parameter("pursuit_point_visualization_topic", kPursuitPointVisualizationTopic);
    this->get_parameter("track_limits_topic", kTrackLimitsTopic);
    this->get_parameter("track_limits_visualization_topic", kTrackLimitsVisualizationTopic);


    triangulation_visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        kTriangulationVisualizationTopic, 10);
    optimized_trajectory_visualization_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        kOptimizedTrajectoryVisualizationTopic, 10);
    arussim_trajectory_visualization_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        kARUSSimTrajectoryVisualizationTopic, 10);
    trajectory_visualization_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        kTrajectoryVisualizationTopic, 10);
    pursuit_point_visualization_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        kPursuitPointVisualizationTopic, 10);
    track_limits_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        kTrackLimitsVisualizationTopic, 10);

    triangulation_sub_ = this->create_subscription<common_msgs::msg::Triangulation>(
        kTriangulationTopic, 10, std::bind(&Visualization::triangulation_callback, this, std::placeholders::_1));
    optimized_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kOptimizedTrajectoryTopic, 10, std::bind(&Visualization::optimized_trajectory_callback, this, std::placeholders::_1));
    arussim_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kARUSSimTrajectoryTopic, 10, std::bind(&Visualization::arussim_trajectory_callback, this, std::placeholders::_1));
    delaunay_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kDelaunayTrajectoryTopic, 10, std::bind(&Visualization::delaunay_trajectory_callback, this, std::placeholders::_1));
    acc_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kAccTrajectoryTopic, 10, std::bind(&Visualization::acc_trajectory_callback, this, std::placeholders::_1));
    skidpad_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kSkidpadTrajectoryTopic, 10, std::bind(&Visualization::skidpad_trajectory_callback, this, std::placeholders::_1));
    pursuit_point_sub_ = this->create_subscription<common_msgs::msg::PointXY>(
        kPursuitPointTopic, 10, std::bind(&Visualization::pursuit_point_callback, this, std::placeholders::_1));
    track_limits_sub_ = this->create_subscription<common_msgs::msg::TrackLimits>(
        kTrackLimitsTopic, 10, std::bind(&Visualization::track_limits_callback, this, std::placeholders::_1));

}

void Visualization::triangulation_callback(const common_msgs::msg::Triangulation::SharedPtr msg)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    for (int i = 0; i < msg -> simplices.size(); i++){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "arussim/world";
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
        marker.color.a = kAlpha;
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

void Visualization::track_limits_callback(const common_msgs::msg::TrackLimits::SharedPtr msg)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "arussim/world";
    marker.ns = "left_track_limits";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.a = kAlpha;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    for (int i = 0; i < msg -> left_limit.size(); i++){
        geometry_msgs::msg::Point p;
        p.x = msg->left_limit[i].x;
        p.y = msg->left_limit[i].y;
        p.z = 0;
        marker.points.push_back(p);
    }
    marker.points.push_back(marker.points[0]);
    marker_array.markers.push_back(marker);
    visualization_msgs::msg::Marker marker2;
    marker2.header.frame_id = "arussim/world";
    marker2.ns = "right_track_limits";
    marker2.id = 1;
    marker2.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker2.action = visualization_msgs::msg::Marker::ADD;
    marker2.pose.position.x = 0;
    marker2.pose.position.y = 0;
    marker2.pose.position.z = 0;
    marker2.scale.x = 0.4;
    marker2.scale.y = 0.4;
    marker2.scale.z = 0.4;
    marker2.color.a = kAlpha;
    marker2.color.r = 1.0;
    marker2.color.g = 1.0;
    marker2.color.b = 0.0;
    marker2.lifetime = rclcpp::Duration::from_seconds(0.0);
    for (int i = 0; i < msg -> right_limit.size(); i++){
        geometry_msgs::msg::Point p;
        p.x = msg->right_limit[i].x;
        p.y = msg->right_limit[i].y;
        p.z = 0;
        marker2.points.push_back(p);
    }
    marker2.points.push_back(marker2.points[0]);
    marker_array.markers.push_back(marker2);
    track_limits_vis_pub_->publish(marker_array);

}

void Visualization::arussim_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker = this->create_trajectory_marker(msg, true, 1.0, 0.0, 0.0, kAlpha/2);
    arussim_trajectory_visualization_pub_->publish(marker);
}

void Visualization::optimized_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker = this->create_trajectory_marker(msg, true, 1.0, 0.5, 0.0, kAlpha);
    optimized_trajectory_visualization_pub_->publish(marker);
}

void Visualization::delaunay_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker = this->create_trajectory_marker(msg, true, 1.0, 0.0, 0.0, kAlpha);
    trajectory_visualization_pub_->publish(marker);
}

void Visualization::acc_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker = this->create_trajectory_marker(msg, true, 1.0, 0.0, 0.0, kAlpha);
    trajectory_visualization_pub_->publish(marker);
}

void Visualization::skidpad_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker = this->create_trajectory_marker(msg, true, 1.0, 0.0, 0.0, kAlpha);
    trajectory_visualization_pub_->publish(marker);
}

void Visualization::pursuit_point_callback(const common_msgs::msg::PointXY::SharedPtr msg)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "arussim/world";
    marker.ns = "pursuit_point";
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = msg->x;
    marker.pose.position.y = msg->y;
    marker.pose.position.z = 0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    pursuit_point_visualization_pub_->publish(marker);
}

visualization_msgs::msg::Marker Visualization::create_trajectory_marker(
    const common_msgs::msg::Trajectory::SharedPtr msg, 
    bool global, double red, double green, double blue, double alpha)
{
    visualization_msgs::msg::Marker marker;
    if(global){
        marker.header.frame_id = "arussim/world";
    } else {
        marker.header.frame_id = "arussim/vehicle_cog";
    }
    marker.ns = "trajectory";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = alpha;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
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