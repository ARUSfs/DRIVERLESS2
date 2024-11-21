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

    //Create the publishers
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/filtered_cloud", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/map_cloud", 10);
}

void Perception::get_clusters_centers(std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<PointXYZColorScore>::Ptr map_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
{
    for (const auto &cluster : cluster_indices)
    {
        //Create a temporal point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud_filtered, cluster, *cluster_cloud);

        //Obtain the bounding box of the cluster
        pcl::PointXYZI min_point, max_point;
        pcl::getMinMax3D(*cluster_cloud, min_point, max_point);
        float max_x = max_point.x;
        float min_x = min_point.x;
        float max_y = max_point.y;
        float min_y = min_point.y;
        float max_z = max_point.z;
        float min_z = min_point.z;

        //filter the cluster by size and keep the center of the cloud
        if ((max_z - min_z) > 0.1 && (max_z - min_z) < 0.4 && (max_x - min_x) < 0.4 && (max_y - min_y) < 0.4)
        {
            PointXYZColorScore cone;
            cone.x = (max_x + min_x) / 2;
            cone.y = (max_y + min_y) / 2;
            cone.z = 0;
            cone.color = 0;
            cone.score = 1;
            map_cloud->push_back(cone);
        }
    }
}

/**
 * @brief Create callback function for the lidar topic.
 * @param lidar_msg The point cloud message received from the lidar topic.
 */
void Perception::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{   
    double start_time = this->now().seconds();
    
    //Transform the message into a pcl point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*lidar_msg, *cloud);

    //Crop the point cloud
    Cropping::crop_filter_condition(cloud, kMaxXFov, kMaxYFov, kMaxZFov, kHFov);

    //print the number of filtered points and the time of the cropping function used
    std::cout << "Number of filtered points: " << cloud->size() << std::endl;
    std::cout << "Cropping Time: " << this->now().seconds() - start_time << std::endl;

    //Define the variables for the ground filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    //Apply the ground filter fuction
    GroundFiltering::ransac_ground_filter(cloud, cloud_filtered, cloud_plane, coefficients, kThreshold);
    
    //Print the time of the ground filter algorithm used
    std::cout << "Ground Filter Time: " << this->now().seconds() - start_time << std::endl;

    //Extract the clusters from the point cloud
    std::vector<pcl::PointIndices> cluster_indices;
    Clustering::euclidean_clustering(cloud_filtered, cluster_indices);

    //Store the clusters centers in a new point cloud
    pcl::PointCloud<PointXYZColorScore>::Ptr map_cloud(new pcl::PointCloud<PointXYZColorScore>);
    Perception::get_clusters_centers(cluster_indices, map_cloud, cloud_filtered);

    //Print the number of cones
    std::cout << "Number of cones: " << map_cloud->size() << std::endl;

    //Publish the filtered cloud
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered,filtered_msg);
    filtered_msg.header.frame_id="/rslidar";
    filtered_pub_->publish(filtered_msg);

    //Publish the map cloud
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud,map_msg);
    map_msg.header.frame_id="/rslidar";
    map_pub_->publish(map_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Perception>());
    rclcpp::shutdown();
    return 0;
}