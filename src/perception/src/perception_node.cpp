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
    this->declare_parameter<double>("threshold_ground_filter", 0.05);
    this->declare_parameter<double>("radius", 1.0);
    this->declare_parameter<int>("number_sections", 8);
    this->declare_parameter<double>("angle_threshold", 20);
    this->declare_parameter<int>("number_rings", 6);
    this->declare_parameter<int>("number_sectors", 8);
    this->declare_parameter<double>("max_radius", 25);
    this->declare_parameter<int>("minimum_ransac_points", 30);
    this->declare_parameter<double>("threshold_scoring", 0.4);

    //Get the parameters
    this->get_parameter("lidar_topic", kLidarTopic);
    this->get_parameter("max_x_fov", kMaxXFov);
    this->get_parameter("max_y_fov", kMaxYFov);
    this->get_parameter("max_z_fov", kMaxZFov);
    this->get_parameter("h_fov", kHFov);
    this->get_parameter("threshold_ground_filter", kThresholdGroundFilter);
    this->get_parameter("radius", kRadius);
    this->get_parameter("number_sections", kNumberSections);
    this->get_parameter("angle_threshold", kAngleThreshold);
    this->get_parameter("number_rings", kNumberRings);
    this->get_parameter("number_sectors", kNumberSectors);
    this->get_parameter("max_radius", kMaxRadius);
    this->get_parameter("minimum_ransac_points", kMinimumRansacPoints);
    this->get_parameter("threshold_scoring", kThresholdScoring);

    //Transform into radians
    kHFov *= (M_PI/180);
    
    //Create the subscriber
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&Perception::lidar_callback, this, std::placeholders::_1));

    //Create the publishers
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/filtered_cloud", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/map", 10);
    map_pub_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/map2", 10);
}

/**
 * @brief Extract the center of each cluster.
 * @param cluster_indices The indices of the points that form each cluster.
 * @param cloud_filtered The input point cloud.
 * @param cluster_centers The center of each cluster.
 */
void Perception::get_clusters_centers(std::vector<pcl::PointIndices>& cluster_indices,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& clusters_centers)
{
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); )
    {
        //Create a temporal point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud_filtered, *it, *cluster_cloud);

        //Obtain the new bounding box of the cluster
        pcl::PointXYZI min_point, max_point;
        pcl::getMinMax3D(*cluster_cloud, min_point, max_point);
        double max_x = max_point.x;
        double min_x = min_point.x;
        double max_y = max_point.y;
        double min_y = min_point.y;
        double max_z = max_point.z;
        double min_z = min_point.z;

        //Filter the cluster by size and keep the center of the cluster
        //if ((max_z - min_z) > 0.1 && (max_z - min_z) < 0.4 && (max_x - min_x) < 0.4 && (max_y - min_y) < 0.4)
        if (true)
        {
            PointXYZColorScore center;
            center.x = (max_x + min_x) / 2;
            center.y = (max_y + min_y) / 2;
            center.z = (max_z + min_z) / 2;
            center.color = 0;
            center.score = 1;
            clusters_centers.push_back(center);

            it++;
        }
        else
        {
            it = cluster_indices.erase(it);
        }
    }
    //Resize the cluster indices vector
    cluster_indices.resize(clusters_centers.size());
}

/**
 * @brief Recover falsely ground filtered points.
 * @param cloud_plane The input point cloud.
 * @param cloud_filtered The filtered point cloud.
 * @param cluster_indices The indices of the points that form each cluster.
 * @param cluster_centers The center of each cluster.
 * @param radius The radius used to search for eliminated points.
 */
void Perception::reconstruction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, 
    std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore> clusters_centers, 
    double radius)
{
    //Iterate on clusters
    for (size_t i = 0; i < clusters_centers.size(); ++i)
    {
        //Convert from PointXYZColorScore to PointXYZI
        pcl::PointXYZI center;
        center.x = clusters_centers[i].x;
        center.y = clusters_centers[i].y;
        center.z = clusters_centers[i].z;
        center.intensity = clusters_centers[i].score;
        
        //Iterate on planar points
        for (size_t j = 0; j < cloud_plane->size(); ++j)
        {
            pcl::PointXYZI point = cloud_plane->points[j];

            //Check if the point lies inside the cylinder
            double dx = point.x - center.x;
            double dy = point.y - center.y;
            double radial_distance = std::sqrt(dx * dx + dy * dy);

            //Insert the recovered points
            if (radial_distance <= radius)
            {
                //Add the point to the filtered cloud
                cloud_filtered->points.push_back(point);
                cloud_filtered->width++;

                //Add the index of the recovered point to cluster indices
                cluster_indices[i].indices.push_back(cloud_filtered->points.size() - 1);
            }
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
    Cropping::crop_filter_cropbox(cloud, kMaxXFov, kMaxYFov, kMaxZFov);

    //print the number of filtered points and the time of the cropping function used
    //std::cout << "Cropping Time: " << this->now().seconds() - start_time << std::endl;

    //Define the variables for the ground filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    //Apply the ground filter fuction
    GroundFiltering::grid_ground_filter(cloud, cloud_filtered, cloud_plane, coefficients, kThresholdGroundFilter, kMaxXFov, kMaxYFov, kMaxZFov, kNumberSections, kAngleThreshold, kMinimumRansacPoints);
    
    //Print the time of the ground filter algorithm used
    std::cout << "Ground Filter Time: " << this->now().seconds() - start_time << std::endl;

    //Extract the clusters from the point cloud
    std::vector<pcl::PointIndices> cluster_indices;
    Clustering::euclidean_clustering(cloud_filtered, cluster_indices);

    //Store the clusters centers in a new point cloud
    std::vector<PointXYZColorScore> clusters_centers;
    Perception::get_clusters_centers(cluster_indices, cloud_filtered, clusters_centers); 

    //Print the number of possibles cones
    //std::cout << "Number of posibles cones: " << clusters_centers.size() << std::endl;

    //Recover ground points
    Perception::reconstruction(cloud_plane, cloud_filtered, cluster_indices, clusters_centers, kRadius);
    
    //Print the number of recovered points
    //std::cout << "Reconstruction time: " << this->now().seconds() - start_time << std::endl;

    //Score the clusters and keep the ones that will be consider cones
    pcl::PointCloud<PointXYZColorScore>::Ptr final_map(new pcl::PointCloud<PointXYZColorScore>);
    Scoring::scoring_surface(cloud_filtered, final_map, cluster_indices, clusters_centers, kThresholdScoring);

    //Print the number of cones and the time of the scoring
    //std::cout << "Number of cones: " << final_map->size() << std::endl;
    //std::cout << "Scoring time: " << this->now().seconds() - start_time << std::endl;

    //Estimate the color of each cone
    //ColorEstimation::coloring(cluster_indices, clusters_centers, cloud_filtered);

    //Print the time of the coloring
    //std::cout << "coloring time: " << this->now().seconds() - start_time << std::endl;

    pcl::PointCloud<PointXYZColorScore>::Ptr final_map2(new pcl::PointCloud<PointXYZColorScore>);
    for (size_t i = 0; i < clusters_centers.size(); ++i)
    {
        PointXYZColorScore& center = clusters_centers[i];

        if (center.color == 4)
        {
            final_map2->points.push_back(center);
        }
    }

    std::cout << "////////////////////////////////////////////////////////// " << std::endl;

    //Publish the filtered cloud
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_msg);
    filtered_msg.header.frame_id="/rslidar";
    filtered_pub_->publish(filtered_msg);

    //Publish the map cloud
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*final_map, map_msg);
    map_msg.header.frame_id="/rslidar";
    map_pub_->publish(map_msg);

    sensor_msgs::msg::PointCloud2 map_msg2;
    pcl::toROSMsg(*final_map2, map_msg2);
    map_msg2.header.frame_id="/rslidar";
    map_pub_2->publish(map_msg2);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Perception>());
    rclcpp::shutdown();
    return 0;
}