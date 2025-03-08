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
    // Transform the message into a pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*lidar_msg, *cloud);

    // Definme the plane of the proeyction of the point cloud
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = 0.0;  
    coefficients->values[1] = 0.0;  
    coefficients->values[2] = 1.0;  
    coefficients->values[3] = 0.0;  

    // Define the proyection
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);

    // Flat the point cloud by proyecting it into the plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_flattened(new pcl::PointCloud<pcl::PointXYZ>);
    proj.filter(*cloud_flattened);

    // Filter those points with NaN values
    std::vector<int> indices_not_NaN;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_flattened_not_NaN(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::removeNaNFromPointCloud(*cloud_flattened, *cloud_flattened_not_NaN, indices_not_NaN);

    // int number_sections = 50;
    // int density_threshold = 15;
    // double Mx = 30;
    // double My = 15;
    // double Mz = 0.5;

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    // double x_step = (Mx - 0) / number_sections;
    // double y_step = (My - (-My)) / number_sections;

    // int cont = 0;

    // for (int i = 0; i < number_sections; ++i)
    // {
    //     for (int j = 0; j < number_sections; ++j)
    //     {
    //         // Define the square
    //         Eigen::Vector4f min_pt(0 + i * x_step, -My + j * y_step, -100.0, 1.0);
    //         Eigen::Vector4f max_pt(0 + (i + 1) * x_step, -My + (j + 1) * y_step, Mz, 1.0);

    //         // Crop the input cloud to the square measures
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //         pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    //         crop_box_filter.setInputCloud(cloud_flattened_not_NaN);
    //         crop_box_filter.setMin(min_pt);
    //         crop_box_filter.setMax(max_pt);
    //         crop_box_filter.filter(*grid_cloud);

    //         double k = (i * j * number_sections) % 256;

    //         for (const auto& p : grid_cloud->points)
    //         {
    //             pcl::PointXYZI copy;
    //             copy.x = p.x;
    //             copy.y = p.y;
    //             copy.z = p.z;
    //             copy.intensity = k;

    //             cloud_filtered->points.push_back(copy);
    //         }

    //         ++cont;

    //         std::cout << "Number of points: " << grid_cloud->points.size() << std::endl;
    //         std::cout << "Intensity: " << k << std::endl;
    //     }
    // }
    // std::cout << "------------> Number of grids " << cont << " <------------" << std::endl;

    // float bandwidth = 0.5;  
    // int min_neighbors = 20;

    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // kdtree.setInputCloud(cloud_flattened_not_NaN);

    // std::vector<float> densities(cloud_flattened_not_NaN->size(), 0);

    // for (size_t i = 0; i < cloud_flattened_not_NaN->size(); ++i) 
    // {
    //     std::vector<int> pointIdxRadiusSearch;
    //     std::vector<float> pointRadiusSquaredDistance;

    //     if (kdtree.radiusSearch(cloud_flattened_not_NaN->points[i], bandwidth, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) 
    //     {
    //         densities[i] = static_cast<float>(pointIdxRadiusSearch.size());
    //     }
    // }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // for (size_t i = 0; i < cloud_flattened_not_NaN->size(); ++i) 
    // {
    //     if (densities[i] >= min_neighbors) 
    //     { 
    //         cloud_filtered->push_back(cloud_flattened_not_NaN->points[i]);
    //     }
    // }

    // std::cout << "Number of filtered points: " << cloud_filtered->size() << std::endl;

    // Publish the filtered cloud
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_flattened_not_NaN, filtered_msg);
    filtered_msg.header.frame_id="/rslidar";
    filtered_pub_->publish(filtered_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NewPerception>());
    rclcpp::shutdown();
    return 0;
}