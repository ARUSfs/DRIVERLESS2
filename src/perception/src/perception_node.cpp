/**
 * @file perception_node.hpp
 * @author Alejandro Vallejo Mayo
 * @brief Contains the main function and the implementation of the methods to achieve a robust and reliable perception algorithm for the ARUS 
 * Team, which extracts the location of the cones on the track.
 */


#include "perception/perception_node.hpp"


Perception::Perception() : Node("Perception")
{
    // Declare parameters
    this->declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    this->declare_parameter<std::string>("state_topic", "/car_state/state");
    this->declare_parameter<bool>("crop", true);
    this->declare_parameter<double>("max_x_fov", 25.0);
    this->declare_parameter<double>("max_y_fov", 20.0);
    this->declare_parameter<double>("max_z_fov", 0.0);
    this->declare_parameter<double>("threshold_ground_filter", 0.05);
    this->declare_parameter<int>("number_sections", 8);
    this->declare_parameter<double>("angle_threshold", 20);
    this->declare_parameter<int>("minimum_ransac_points", 30);
    this->declare_parameter<double>("radius", 1.0);
    this->declare_parameter<double>("threshold_scoring", 0.7);
    this->declare_parameter<double>("distance_threshold", 0.4);
    this->declare_parameter<double>("coloring_threshold", 0.4);
    this->declare_parameter<bool>("debug", true);

    // Get the parameters
    this->get_parameter("lidar_topic", kLidarTopic);
    this->get_parameter("state_topic", kStateTopic);
    this->get_parameter("crop", kCrop);
    this->get_parameter("max_x_fov", kMaxXFov);
    this->get_parameter("max_y_fov", kMaxYFov);
    this->get_parameter("max_z_fov", kMaxZFov);
    this->get_parameter("threshold_ground_filter", kThresholdGroundFilter);
    this->get_parameter("number_sections", kNumberSections);
    this->get_parameter("angle_threshold", kAngleThreshold);
    this->get_parameter("minimum_ransac_points", kMinimumRansacPoints);
    this->get_parameter("radius", kRadius);
    this->get_parameter("threshold_scoring", kThresholdScoring);
    this->get_parameter("distance_threshold", kDistanceThreshold);
    this->get_parameter("coloring_threshold", kColoringThreshold);
    this->get_parameter("debug", kDebug);

    // Initialize the variables
    vx = 0.0;
    vy = 0.0;
    yaw_rate = 0.0;
    dt = 0.1;
    
    // Create the subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&Perception::lidar_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<common_msgs::msg::State>(
            kStateTopic, 10, std::bind(&Perception::state_callback, this, std::placeholders::_1));
    
    // Create the publishers
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/filtered_cloud", 10);
    clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/clusters", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/map", 10);
}


void Perception::state_callback(const common_msgs::msg::State::SharedPtr state_msg)
{
    vx = state_msg->vx;
    vy = state_msg->vy;
    yaw_rate = state_msg->r;
}


void Perception::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{   
    double start_time = this->now().seconds();


    // Define the necessary variables
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<PointXYZColorScore> clusters_centers;
    pcl::PointCloud<pcl::PointXYZI>::Ptr clusters_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointXYZColorScore>::Ptr final_map(new pcl::PointCloud<PointXYZColorScore>);

    
    // Transform the message into a pcl point cloud
    pcl::fromROSMsg(*lidar_msg, *cloud);


    if (kDebug && cloud->size() == 0) 
    {
        RCLCPP_WARN(this->get_logger(), "Empty point cloud");
        return;
    }


    if (kCrop) 
    {
        // Crop the point cloud
        Cropping::crop_filter_cropbox(cloud, kMaxXFov, kMaxYFov, kMaxZFov);
        if (kDebug) RCLCPP_INFO(this->get_logger(), "Cropping Time: %f", this->now().seconds() - start_time);
    }


    // Apply the ground filter function
    GroundFiltering::grid_ground_filter(cloud, cloud_filtered, cloud_plane, coefficients, kThresholdGroundFilter, kMaxXFov, kMaxYFov, 
        kMaxZFov, kNumberSections, kAngleThreshold, kMinimumRansacPoints);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Ground Filter Time: %f", this->now().seconds() - start_time);


    // Extract the clusters from the point cloud
    Clustering::euclidean_clustering(cloud_filtered, cluster_indices);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Clustering time: %f", this->now().seconds() - start_time);


    // Store the clusters centers in a new point cloud
    Utils::get_clusters_centers(cluster_indices, cloud_filtered, clusters_centers);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Center calculation time: %f", this->now().seconds() - start_time);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Number of possible cones: %zu", clusters_centers.size());


    // Recover ground points
    Utils::reconstruction(cloud_plane, cloud_filtered, cluster_indices, clusters_centers, kRadius);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Reconstruction time: %f", this->now().seconds() - start_time);


    // Filter the clusters by size
    Utils::filter_clusters(cluster_indices, cloud_filtered, clusters_centers);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Filtering time: %f", this->now().seconds() - start_time);


    // Convert the indices of the clusters to the points of the clusters
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_points;
    for (const auto& cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr new_cluster(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& idx : cluster.indices)
        {
            pcl::PointXYZI point = cloud_filtered->points[idx];
            new_cluster->points.push_back(point);
        } 
        cluster_points.push_back(new_cluster);
    }


    if (kDebug) 
    {
        // Publish clusters cloud with different colors
        int i = 0;
        for (auto c : cluster_points)
        {
            for (auto &p : c->points)
            {
                pcl::PointXYZI q = p;
                q.intensity = i;
                clusters_cloud->push_back(q);
            }
            i++;
        }
    }


    // Score the clusters and keep the ones that will be considered cones
    Scoring::scoring_surface(final_map, cluster_points, clusters_centers, kThresholdScoring);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Scoring time: %f", this->now().seconds() - start_time);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Number of cones: %zu", final_map->size());


    // Estimate the color of the closest cones
    ColorEstimation::color_estimation(cluster_points, clusters_centers, kDistanceThreshold, kColoringThreshold);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Color estimation time: %f", this->now().seconds() - start_time);


    // Update the colors of final map points
    for (auto& point : final_map->points) 
    {
        for (const auto& center : clusters_centers) 
        {
            if (point.x == center.x && point.y == center.y && point.z == center.z) 
            {
                point.color = center.color;
            }
        }
    }


    // Motion correction
    double dt = this->now().seconds() - start_time; // Estimate SDK delay
    Utils::motion_correction(final_map, vx, vy, yaw_rate, dt);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Motion correction time: %f", this->now().seconds() - start_time);


    if (kDebug) RCLCPP_INFO(this->get_logger(), "//////////////////////////////////////////////");


    if (kDebug)
    {
        // Publish the filtered cloud
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_filtered, filtered_msg);
        filtered_msg.header.frame_id = "/rslidar";
        filtered_pub_->publish(filtered_msg);

        // Publish the clusters cloud
        sensor_msgs::msg::PointCloud2 clusters_msg;
        pcl::toROSMsg(*clusters_cloud, clusters_msg);
        clusters_msg.header.frame_id = "/rslidar";
        clusters_pub_->publish(clusters_msg);
    }


    // Publish the map cloud
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*final_map, map_msg);
    map_msg.header.frame_id = "/rslidar";
    map_pub_->publish(map_msg);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Perception>());
    rclcpp::shutdown();
    return 0;
}