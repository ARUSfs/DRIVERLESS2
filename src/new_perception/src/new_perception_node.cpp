/**
 * @file new_perception_node.hpp
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Main file for the NewPerception node. 
 * Contains the main function and the implementation of the methods to achieve a robust and reliable perception algorithm for the ARUS 
 * Team, which extracts the location of the cones on the track.
 * @version 0.1
 * @date 27-2-2025
 */

#include "new_perception/new_perception_node.hpp"

NewPerception::NewPerception() : Node("NewPerception")
{
    // Declare tge parameters
    this->declare_parameter<std::string>("lidar_topic", "/rslidar_points");

    // Get the parameters
    this->get_parameter("lidar_topic", kLidarTopic);

    // Create the subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&NewPerception::lidar_callback, this, std::placeholders::_1));

    // Create the publishers
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/filtered_cloud", 10);
}

/**
 * @brief Create callback function for the lidar topic.
 * @param lidar_msg The point cloud message received from the lidar topic.
 */
void NewPerception::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{
    double start_time = this->now().seconds();


    // Transform the message into a pcl point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*lidar_msg, *cloud);


    Cropping::crop_filter_cropbox(cloud, 25.0, 20.0, 0.0);
    std::cout << "Cropping Time: " << this->now().seconds() - start_time << std::endl;


    double radius = 0.25;  

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    for (size_t i = 0; i < cloud->points.size(); ++i) 
    {
        pcl::PointXYZI p = cloud->points[i];
        double distance = std::sqrt(p.x * p.x + p.y * p.y);

        int min_neighbors = std::max(485 * exp(-0.6 * distance), 10.0);

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >= min_neighbors) 
        {   
            inliers->indices.push_back(i);
        }
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    extract.filter(*cloud_filtered);

    std::cout << "Filtering time: " << this->now().seconds() - start_time << std::endl;


    // Publish the filtered cloud
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_msg);
    filtered_msg.header.frame_id="/rslidar";
    filtered_pub_->publish(filtered_msg);

    std::cout << "//////////////////////////////////////" << std::endl;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NewPerception>());
    rclcpp::shutdown();
    return 0;
}