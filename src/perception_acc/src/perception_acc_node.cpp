/**
 * @file perception_acc_node.hpp
 * @author Álvaro Galisteo Bermúdez (galisbermo03@gmail.com)
 * @brief Main file for the Perception Accumulation node. 
 * Contains the main function and the implementation of the methods to achieve a robust and reliable perception algorithm for the ARUS 
 * Team, which extracts the location of the cones on the track.
 * @version 0.1
 * @date 11-3-2025
 */

#include "perception_acc/perception_acc_node.hpp"
bool DEBUG = false;

Perception::Perception() : Node("Perception")
{
    //Declare the parameters
    this->declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    this->declare_parameter<bool>("crop", true);
    this->declare_parameter<double>("max_x_fov", 25.0);
    this->declare_parameter<double>("max_y_fov", 20.0);
    this->declare_parameter<double>("max_z_fov", 0.0);
    this->declare_parameter<double>("threshold_ground_filter", 0.05);
    this->declare_parameter<double>("radius", 1.0);
    this->declare_parameter<int>("number_sections", 8);
    this->declare_parameter<double>("angle_threshold", 20);
    this->declare_parameter<int>("minimum_ransac_points", 30);
    this->declare_parameter<double>("threshold_scoring", 0.7);
    this->declare_parameter<double>("distance_threshold", 0.4);
    this->declare_parameter<double>("coloring_threshold", 0.4);
    this->declare_parameter<bool>("global_accumulation", true);
    this->declare_parameter<double>("distance_lidar_to_CoG", 1.65);
    this->declare_parameter<float>("downsample_size", 0.05);

    //Get the parameters
    this->get_parameter("lidar_topic", kLidarTopic);
    this->get_parameter("crop", kCrop);
    this->get_parameter("max_x_fov", kMaxXFov);
    this->get_parameter("max_y_fov", kMaxYFov);
    this->get_parameter("max_z_fov", kMaxZFov);
    this->get_parameter("threshold_ground_filter", kThresholdGroundFilter);
    this->get_parameter("radius", kRadius);
    this->get_parameter("number_sections", kNumberSections);
    this->get_parameter("angle_threshold", kAngleThreshold);
    this->get_parameter("minimum_ransac_points", kMinimumRansacPoints);
    this->get_parameter("threshold_scoring", kThresholdScoring);
    this->get_parameter("distance_threshold", kDistanceThreshold);
    this->get_parameter("coloring_threshold", kColoringThreshold);
    this->get_parameter("global_accumulation", kGlobalAccumulation);
    this->get_parameter("distance_lidar_to_CoG", kDistanceLidarToCoG);
    this->get_parameter("downsample_size", kDownsampleSize);

    //Transform into radians
    kHFov *= (M_PI/180);
    
    //Create the subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&Perception::lidar_callback, this, std::placeholders::_1));
    
    //Create the publishers
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/filtered_cloud", 10);
    clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/clusters", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/map", 10);

    // Inicializa el tf_buffer y tf_listener de manera que se suscriban al topic /tf
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
        if ((max_z-min_z)<0.4 && min_z<0.0 && (max_x-min_x)<0.5 && (max_y-min_y)<0.5)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster_cloud, centroid);
            PointXYZColorScore center;
            center.x = centroid[0];
            center.y = centroid[1];
            center.z = min_z;
            center.color = 0;
            center.score = 0;
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
 * @brief Filter the final clusters by size to delete the ones that are too small or too large to be considered cones.
 * @param cluster_indices The indices of the points that form each cluster.
 * @param cloud_filtered The input point cloud.
 * @param cluster_centers The center of each cluster.
 */
void Perception::filter_clusters(std::vector<pcl::PointIndices>& cluster_indices,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& clusters_centers)
{
    for (int i = cluster_indices.size() - 1; i >= 0; i--)
    {
        //Create a temporal point cloud
        pcl::PointIndices indices = cluster_indices[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud_filtered, indices, *cluster_cloud);

        //Obtain the new bounding box of the cluster
        pcl::PointXYZI min_point, max_point;
        pcl::getMinMax3D(*cluster_cloud, min_point, max_point);
        double max_x = max_point.x;
        double min_x = min_point.x;
        double max_y = max_point.y;
        double min_y = min_point.y;
        double max_z = max_point.z;
        double min_z = min_point.z;

        //Filter the cluster by size and delete the ones that are too small or too large
        if ((max_z - min_z) < 0.10 || (max_z - min_z) > 0.4 || (max_x - min_x) > 0.5 || (max_y - min_y) > 0.5)
        {
            clusters_centers.erase(clusters_centers.begin() + i);
            cluster_indices.erase(cluster_indices.begin() + i);
        }
    }
    
    //Resize the vectors
    clusters_centers.resize(clusters_centers.size());
    cluster_indices.resize(clusters_centers.size());
}

/**
 * @brief Create callback function for the lidar topic.
 * @param lidar_msg The point cloud message received from the lidar topic.
 */
void Perception::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{   
    this->get_tf_position();

    //Define the variables for the ground filter
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::fromROSMsg(*lidar_msg, *cloud);
    
    float zmin = -1.0;   
    float zmax = 0.2;    

    pcl::PointCloud<PointXYZIRingTime>::Ptr filtered_cloud(new pcl::PointCloud<PointXYZIRingTime>);

    for (const auto& point : cloud->points) {
        if (point.z >= zmin && point.z <= zmax) {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    cloud = filtered_cloud;

    pcl::PointCloud<PointXYZIRingTime>::Ptr updated_cloud;
    if (kGlobalAccumulation) updated_cloud = Accumulation::accumulate_global_cloud_ring(cloud, x_, y_, yaw_, kDistanceLidarToCoG, kDownsampleSize);
    else updated_cloud = Accumulation::accumulate_local_cloud_ring(cloud, x_, y_, yaw_, kDistanceLidarToCoG, kDownsampleSize);

    RCLCPP_INFO(this->get_logger(), "Accumulated cloud size: %zu", updated_cloud->size());

    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered_ground(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    GroundRemove::RemoveGround(*updated_cloud,*cloud_filtered_ground,*cloud_filtered); 
    //GroundFiltering::grid_ground_filter(updated_cloud, cloud_filtered, cloud_filtered_ground, coefficients, kThresholdGroundFilter, kMaxXFov, kMaxYFov, kMaxZFov, kNumberSections, kAngleThreshold, kMinimumRansacPoints);

    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_msg);
    filtered_msg.header.frame_id="/rslidar";
    filtered_pub_->publish(filtered_msg);
}

/**
 * @brief Get the position of the car in the world frame.
 * * This function uses the tf2 library to get the transform between the car and the world frame.
 * * It uses the tf_buffer_ to get the transform and then it extracts the position and orientation of the car.
 * 
 */
void Perception::get_tf_position()
{
    geometry_msgs::msg::TransformStamped transform;
    try {
        // Graph slam tf
        transform = tf_buffer_->lookupTransform("arussim/world", "slam/vehicle", tf2::TimePointZero);
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        x_ = transform.transform.translation.x;
        y_ = transform.transform.translation.y;
        yaw_ = yaw;
        
        RCLCPP_INFO(this->get_logger(), "Transform: x: %f, y: %f, yaw: %f", x_, y_, yaw_);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform not available: %s", ex.what());
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Perception>());
    rclcpp::shutdown();
    return 0;
}