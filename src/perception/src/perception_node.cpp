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
    this->declare_parameter<std::string>("perception_topic", "/perception/map");
    this->declare_parameter<std::string>("accum_cloud_topic", "/perception/accumulated_cloud");
    this->declare_parameter<std::string>("filtered_cloud_topic", "/perception/filtered_cloud");
    this->declare_parameter<std::string>("clusters_cloud_topic", "/perception/clusters_cloud");
    this->declare_parameter<double>("lidar_cog_x_offset", 1.5);
    this->declare_parameter<int>("accum_buffer_size", 10);
    this->declare_parameter<bool>("voxel_filter", false);
    this->declare_parameter<double>("voxel_size_x", 0.1);
    this->declare_parameter<double>("voxel_size_y", 0.1);
    this->declare_parameter<double>("voxel_size_z", 0.1);
    this->declare_parameter<bool>("crop", true);
    this->declare_parameter<double>("max_x_fov", 30.0);
    this->declare_parameter<double>("max_y_fov", 15.0);
    this->declare_parameter<double>("max_z_fov", 0.5);
    this->declare_parameter<std::string>("ground_filter_type", "Z_FILTER");
    this->declare_parameter<double>("threshold_ground_filter", 0.075);
    this->declare_parameter<int>("number_sections", 8);
    this->declare_parameter<double>("angle_threshold", 35.0);
    this->declare_parameter<int>("min_cluster_size", 4);
    this->declare_parameter<int>("max_cluster_size", 200);
    this->declare_parameter<double>("tolerance", 0.5);
    this->declare_parameter<bool>("reconstruction", true);
    this->declare_parameter<double>("reconstruction_radius", 0.15);
    this->declare_parameter<double>("threshold_scoring", 0.7);
    this->declare_parameter<bool>("color", true);
    this->declare_parameter<double>("distance_threshold", 5.0);
    this->declare_parameter<double>("coloring_threshold", 15.0);
    this->declare_parameter<bool>("debug", true);

    // Get the parameters
    this->get_parameter("lidar_topic", kLidarTopic);
    this->get_parameter("state_topic", kStateTopic);
    this->get_parameter("perception_topic", kPerceptionTopic);
    this->get_parameter("accum_cloud_topic", kAccumCloudTopic);
    this->get_parameter("filtered_cloud_topic", kFilteredCloudTopic);
    this->get_parameter("clusters_cloud_topic", kClustersCloudTopic);
    this->get_parameter("lidar_cog_x_offset", kLidarCogX);
    this->get_parameter("accum_buffer_size", kAccumBufferSize);
    this->get_parameter("voxel_filter", kVoxelFilter);
    this->get_parameter("voxel_size_x", kVoxelSizeX);
    this->get_parameter("voxel_size_y", kVoxelSizeY);
    this->get_parameter("voxel_size_z", kVoxelSizeZ);
    this->get_parameter("crop", kCrop);
    this->get_parameter("max_x_fov", kMaxXFov);
    this->get_parameter("max_y_fov", kMaxYFov);
    this->get_parameter("max_z_fov", kMaxZFov);
    this->get_parameter("ground_filter_type", kGroundFilterType);
    this->get_parameter("threshold_ground_filter", kThresholdGroundFilter);
    this->get_parameter("number_sections", kNumberSections);
    this->get_parameter("angle_threshold", kAngleThreshold);
    this->get_parameter("min_cluster_size", kMinClusterSize);
    this->get_parameter("max_cluster_size", kMaxClusterSize);
    this->get_parameter("tolerance", kTolerance);
    this->get_parameter("reconstruction", kReconstruction);
    this->get_parameter("reconstruction_radius", kRecRadius);
    this->get_parameter("threshold_scoring", kThresholdScoring);
    this->get_parameter("color", kColor);
    this->get_parameter("distance_threshold", kDistanceThreshold);
    this->get_parameter("coloring_threshold", kColoringThreshold);
    this->get_parameter("debug", kDebug);

    // Initialize the variables
    vx_ = 0.0;
    vy_ = 0.0;
    r_ = 0.0;
    dt = 0.1;

    prev_cones_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    acum_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // Create the subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&Perception::lidar_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<common_msgs::msg::State>(
            kStateTopic, 10, std::bind(&Perception::state_callback, this, std::placeholders::_1));

    // Create the publishers
    accumulation_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        kAccumCloudTopic, 10);
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        kFilteredCloudTopic, 10);
    clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        kClustersCloudTopic, 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        kPerceptionTopic, 10);
}



void Perception::state_callback(const common_msgs::msg::State::SharedPtr state_msg)
{
    vx_ = state_msg->vx;
    vy_ = state_msg->vy;
    r_ = state_msg->r;
}


void Perception::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{
    double start_time = this->now().seconds();


    // Transform the message into a pcl point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*lidar_msg, *cloud);
    Eigen::Matrix4f T_lidar_to_cog = Eigen::Matrix4f::Identity();
    T_lidar_to_cog(0, 3) = kLidarCogX; // x translation
    pcl::transformPointCloud(*cloud, *cloud, T_lidar_to_cog);

    if (cloud->size() == 0)
    {
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Empty point cloud");
        return;
    }

    
    if (kAccumBufferSize > 0) {
        this->accumulate(cloud);
        if (kDebug) {
            // Publish the accumulation cloud
            sensor_msgs::msg::PointCloud2 acum_msg;
            pcl::toROSMsg(*acum_cloud_, acum_msg);
            acum_msg.header.frame_id = "/rslidar";
            accumulation_pub_->publish(acum_msg);

            RCLCPP_INFO(this->get_logger(), "Accumulation time: %f", this->now().seconds() - start_time);
        }

        cloud = acum_cloud_; // Use the accumulated cloud for processing
    }

    if (kCrop)
    {
        // Crop the point cloud
        double min_x = 0.0;
        double min_y = -kMaxYFov;
        double min_z = -1.0;
        Cropping::box_filter<pcl::PointXYZI>(cloud, cloud, min_x, min_y, min_z, kMaxXFov, kMaxYFov, kMaxZFov);
        if (kDebug) RCLCPP_INFO(this->get_logger(), "Cropping Time: %f", this->now().seconds() - start_time);
    }


    // Apply the ground filter function
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (kGroundFilterType == "Z_FILTER") {
        Cropping::axis_filter<pcl::PointXYZI>(cloud, cloud_filtered, "z", 0.08, 0.4);
    } else if (kGroundFilterType == "RANSAC") {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        GroundFiltering::ransac_ground_filter(cloud, cloud_filtered, ground_cloud, coefficients, kThresholdGroundFilter);
    } else if (kGroundFilterType == "RANSAC_GRID") {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        GroundFiltering::grid_ground_filter(cloud, cloud_filtered, ground_cloud, coefficients, kThresholdGroundFilter,
                                            kMaxXFov, kMaxYFov, kMaxZFov, kNumberSections, kAngleThreshold);
    } else if (kGroundFilterType == "PILLAR") {
        GroundFiltering::pillar_ground_filter(cloud, cloud_filtered, ground_cloud, kThresholdGroundFilter, 
                                              kMaxXFov, kMaxYFov, kNumberSections);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown ground filter type: %s", kGroundFilterType.c_str());
        return;
    }
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Ground Filter Time: %f", this->now().seconds() - start_time);


    // Extract the clusters from the point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_clouds;
    Clustering::FEC<pcl::PointXYZI>(cloud_filtered, &cluster_clouds, kTolerance, kMinClusterSize, kMaxClusterSize + 100*kAccumBufferSize);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Clustering time: %f", this->now().seconds() - start_time);


    if (kDebug && cluster_clouds.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No clusters found");
        return;
    }

    std::vector<pcl::PointXYZI> clusters_centers;
    Utils::get_clusters_centers(&cluster_clouds, &clusters_centers);
    

    if (kReconstruction && !ground_cloud->empty())
    {   
        // Filter too big clusters
        Utils::filter_clusters(&cluster_clouds, &clusters_centers, 0.0, 0.4, 0.5, 1.0);

        // Recover ground points
        Utils::reconstruction(ground_cloud, &cluster_clouds, &clusters_centers, kRecRadius);
        if (kDebug) RCLCPP_INFO(this->get_logger(), "Reconstruction time: %f", this->now().seconds() - start_time);
    }
    
    // Filter too big and too small clusters
    Utils::filter_clusters(&cluster_clouds, &clusters_centers);


    // Score the clusters and keep the ones that will be considered cones
    pcl::PointCloud<PointXYZProbColorScore>::Ptr final_map(new pcl::PointCloud<PointXYZProbColorScore>);
    Scoring::scoring_surface(final_map, cluster_clouds, clusters_centers, kThresholdScoring);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Scoring time: %f", this->now().seconds() - start_time);


    if (kDebug && final_map->size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "No cones found");
        return;
    }


    // Estimate the color of the closest cones
    if (kColor)
    {
        Coloring::color_estimation(final_map, cluster_clouds, clusters_centers, kDistanceThreshold, kDebug);
        if (kDebug) RCLCPP_INFO(this->get_logger(), "Color estimation time: %f", this->now().seconds() - start_time);
    }


    // Motion correction
    double dt = this->now().seconds() - start_time; // Estimate SDK delay
    // Remove or adapt this if Utils::motion_correction expects a different point type
    // Utils::motion_correction(final_map, vx_, vy_, r_, dt);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Motion correction time: %f", this->now().seconds() - start_time);


    final_times.push_back(this->now().seconds() - start_time);
    double average_time = std::accumulate(final_times.begin(), final_times.end(), 0.0) / final_times.size();
    if (kDebug && final_map->size() > 0) RCLCPP_INFO(this->get_logger(), "Number of cones: %zu", final_map->size());
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Final iteration time: %f", this->now().seconds() - start_time);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Average time: %f", average_time);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "//////////////////////////////////////////////");


    if (kDebug)
    {
        // Publish the filtered cloud
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_filtered, filtered_msg);
        filtered_msg.header.frame_id = "/rslidar";
        filtered_pub_->publish(filtered_msg);

        // Publish the clusters cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr all_clusters_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& c : cluster_clouds) *all_clusters_cloud += *c;
        sensor_msgs::msg::PointCloud2 clusters_msg;
        pcl::toROSMsg(*all_clusters_cloud, clusters_msg);
        clusters_msg.header.frame_id = "/rslidar";
        clusters_pub_->publish(clusters_msg);
    }


    // Publish the map cloud
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*final_map, map_msg);
    map_msg.header.frame_id = "/rslidar";
    map_pub_->publish(map_msg);
}



void Perception::accumulate(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){
    // Get ground plane with RANSAC and align the cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr actual_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr actual_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    GroundFiltering::ransac_ground_filter(cloud,  actual_cloud_filtered,  actual_ground_cloud,  coefficients, 0.05);
    Utils::ground_align(cloud, coefficients);

    // Filter the ground points
    Cropping::axis_filter<pcl::PointXYZI>(cloud, cloud, "z", 0.08, 1.0);

    // Extract clusters estimation
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_clouds;
    Clustering::FEC<pcl::PointXYZI>(cloud, &cluster_clouds, kTolerance, kMinClusterSize, kMaxClusterSize);

    std::vector<pcl::PointXYZI> clusters_centers;
    Utils::get_clusters_centers(&cluster_clouds, &clusters_centers);
    Utils::filter_clusters(&cluster_clouds, &clusters_centers);

    // Get cones estimation
    pcl::PointCloud<pcl::PointXYZ>::Ptr actual_cones(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto center : clusters_centers) actual_cones->points.push_back(pcl::PointXYZ(center.x, center.y, center.z));

    if (cloud_buffer_.size() == 0)
    {
        cloud_buffer_.push_back(cloud);
        *prev_cones_ = *actual_cones;
        return;
    }


    // Update previous cones estimation
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = -vx_ * dt; // x translation
    transform(1, 3) = -vy_ * dt; // y translation
    transform(2, 3) = 0.0; // z translation (assuming no vertical movement)
    double phi = - r_ * dt; // rotation around z-axis
    transform(0, 0) = cos(phi);
    transform(0, 1) = -sin(phi);
    transform(1, 0) = sin(phi);
    transform(1, 1) = cos(phi);
    pcl::transformPointCloud(*prev_cones_, *prev_cones_, transform);


    // Pair the previous cones with the actual cones using a KdTree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(actual_cones);

    pcl::PointCloud<pcl::PointXYZ>::Ptr paired_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr paired_dst(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto& pt : prev_cones_->points) {
        std::vector<int> indices(1);
        std::vector<float> sqr_dists(1);

        if (kdtree.nearestKSearch(pt, 1, indices, sqr_dists) > 0) {
            if (sqr_dists[0] <= 1.0f) { // 1 metro al cuadrado
                pt.z = 0;
                pcl::PointXYZ pt_dest = actual_cones->points[indices[0]];
                pt_dest.z = 0; // Height at 0 to ensure the transformation is in the XY plane
                paired_src->points.push_back(pt);
                paired_dst->points.push_back(pt_dest);
            }
        }
    }

    *prev_cones_ = *actual_cones;

    // Estimate the transformation between the two point clouds with SVD
    Eigen::Matrix4f transform2;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*paired_src, *paired_dst, transform2);


    // Accumulate the point cloud using motion and SVD transformations
    acum_cloud_->clear();
    for (auto& pcl : cloud_buffer_)
    {
        pcl::PointCloud<PointXYZProbColorScore>::Ptr cones_map(new pcl::PointCloud<PointXYZProbColorScore>);
        pcl::transformPointCloud(*pcl, *pcl, transform);
        pcl::transformPointCloud(*pcl, *pcl, transform2);
        *acum_cloud_ += *pcl;
    }


    if (cloud_buffer_.size() > kAccumBufferSize)
    {
        cloud_buffer_.pop_back();
    }
    cloud_buffer_.insert(cloud_buffer_.begin(), cloud);

    if (kVoxelFilter) {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(acum_cloud_);
        voxel_filter.setLeafSize(kVoxelSizeX, kVoxelSizeY, kVoxelSizeZ);
        voxel_filter.filter(*acum_cloud_);
    }
    
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Perception>());
    rclcpp::shutdown();
    return 0;
}