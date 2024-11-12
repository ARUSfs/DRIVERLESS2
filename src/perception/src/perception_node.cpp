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
    this->declare_parameter<double>("max_x_fov", 25.0);
    this->declare_parameter<double>("max_y_fov", 20.0);
    this->declare_parameter<double>("max_z_fov", 0.0);
    this->declare_parameter<double>("h_fov", 180.0);
    this->declare_parameter<double>("threshold", 0.05);

    //Get the parameters
    this->get_parameter("lidar_topic", kLidarTopic);
    this->get_parameter("max_x_fov", kMaxXFov);
    this->get_parameter("max_y_fov", kMaxYFov);
    this->get_parameter("max_z_fov", kMaxZFov);
    this->get_parameter("h_fov", kHFov);
    this->get_parameter("threshold", kThreshold);

    //Transform into radians
    kHFov *= (M_PI/180);
    
    //Create the subscriber
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&Perception::lidar_callback, this, std::placeholders::_1));

    //Create the publisher
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/filtered_cloud", 10);
}

/**
 * @brief Create callback function for the lidar topic.
 * @param lidar_msg The point cloud message received from the lidar topic.
 */
void Perception::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{   
    //Keep the time at the start of the function
    rclcpp::Clock kClock;
    rclcpp::Time kInicialTime = kClock.now();
    
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

    //Define the variables for the ground filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    //Apply the ransac ground filter fuction
    GroundFiltering::ransac_ground_filter(cloud, cloud_filtered, cloud_plane, coefficients, kThreshold);

    //Calculate the duration of the ground filter fuction
    rclcpp::Time kFinalGroundFilterTime = kClock.now();
    rclcpp::Duration kGroundFilterTime = kFinalGroundFilterTime - kInicialTime;

    //Publish the filtered cloud
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered,filtered_msg);
    filtered_msg.header.frame_id="/rslidar";
    filtered_pub_->publish(filtered_msg);
    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Perception>());
    rclcpp::shutdown();
    return 0;
}