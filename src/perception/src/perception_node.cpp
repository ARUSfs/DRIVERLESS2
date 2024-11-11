/**
 * @file perception_node.hpp
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Main file for the Perception node. 
 * Contains the main function and the implementation of the methods to achieve a robust and reliable perception algorithm for the ARUS 
 * Team, which extracts the location of the cones on the track.
 * @version 0.1
 * @date 3-11-2024
 */

#include "perception/perception_node.hpp"

Perception::Perception() : Node("Perception")
{
    //Declare the parameters
    this->declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    this->declare_parameter<double>("kMaxXFov", 25.0);
    this->declare_parameter<double>("kMaxYFov", 20.0);
    this->declare_parameter<double>("kMaxZFov", 0.0);
    this->declare_parameter<double>("kHFov", 180.0);

    //Get the parameters
    this->get_parameter("lidar_topic", kLidarTopic);
    this->get_parameter("kMaxXFov", kMaxXFov);
    this->get_parameter("kMaxYFov", kMaxYFov);
    this->get_parameter("kMaxZFov", kMaxZFov);
    this->get_parameter("kHFov", kHFov);

    //Transform 
    kHFov *= (M_PI/180);
    
    //Create the subscriber
    lidar_sub_ = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&Perception::lidar_callback, this, std::placeholders::_1));
}

/**
 * @brief Create callback function for the lidar topic.
 * @param lidar_msg The point cloud message received from the lidar topic.
 */
void Perception::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{   
    //Transform the message into a pcl point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*lidar_msg, *cloud);

    //Crop the point cloud
    double Mx = kMaxXFov;
    double My = kMaxYFov;
    double Mz = kMaxZFov;
    double H = kHFov;
    auto condition = [Mx, My, Mz, H](const pcl::PointXYZI &point)
    {
        return !(point.x < Mx && abs(point.y) < My && point.z < Mz 
        && abs(atan2(point.y, point.x)) < H / 2 && (abs(point.y) > 0.8 || point.x > 2));
    };
    cloud->erase(std::remove_if(cloud->points.begin(), cloud->points.end(), condition), cloud->points.end());
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Perception>());
    rclcpp::shutdown();
    return 0;
}