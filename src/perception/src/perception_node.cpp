/**
 * @file perception_node.hpp
 * @author Alejandro Vallejo Mayo
 * @brief Contains the main function and the implementation of the methods to achieve a robust and reliable perception algorithm for the ARUS
 * Team, which extracts the location of the cones on the track.
 */

#include "perception/perception_node.hpp"


int BUFF_SIZE = 10;


Perception::Perception() : Node("Perception")
{
    // Declare parameters
    this->declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    this->declare_parameter<std::string>("state_topic", "/car_state/state");
    this->declare_parameter<bool>("crop", true);
    this->declare_parameter<double>("max_x_fov", 30.0);
    this->declare_parameter<double>("max_y_fov", 15.0);
    this->declare_parameter<double>("max_z_fov", 0.5);
    this->declare_parameter<double>("threshold_ground_filter", 0.075);
    this->declare_parameter<int>("number_sections", 8);
    this->declare_parameter<double>("angle_threshold", 35.0);
    this->declare_parameter<int>("min_cluster_size", 4);
    this->declare_parameter<int>("max_cluster_size", 200);
    this->declare_parameter<double>("radius", 0.15);
    this->declare_parameter<double>("threshold_scoring", 0.7);
    this->declare_parameter<bool>("color", true);
    this->declare_parameter<double>("distance_threshold", 5.0);
    this->declare_parameter<double>("coloring_threshold", 15.0);
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
    this->get_parameter("min_cluster_size", kMinClusterSize);
    this->get_parameter("max_cluster_size", kMaxClusterSize);
    this->get_parameter("radius", kRadius);
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

    // Create the subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&Perception::lidar_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<common_msgs::msg::State>(
            kStateTopic, 10, std::bind(&Perception::state_callback, this, std::placeholders::_1));

    // Create the publishers
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/filtered_cloud", 10);
    accumulation_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/accumulation_cloud", 10);
    clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/clusters", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/map2", 10);
}

void Perception::process_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cones_map) {

    // Define the necessary variables
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<PointXYZColorScore> clusters_centers;

    // GroundFiltering::pillar_ground_filter2(cloud, cloud_filtered, cloud_plane, kThresholdGroundFilter, kMaxXFov,
    //                                         kMaxYFov, kNumberSections, ground_grid);

    Cropping::axis_filter<pcl::PointXYZI>(cloud, cloud_filtered, "z", 0.08, 1.0);

    // sensor_msgs::msg::PointCloud2 filtered_msg;
    // pcl::toROSMsg(*cloud_filtered, filtered_msg);
    // filtered_msg.header.frame_id = "/rslidar";
    // filtered_pub_->publish(filtered_msg);

    // FEC
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> cluster_indices2;

    FastEuclideanClustering<pcl::PointXYZI> fec;
    fec.setInputCloud(cloud_filtered);
    fec.setSearchMethod(tree);
    fec.setClusterTolerance(0.5);
    fec.setQuality(0.5);
    fec.setMinClusterSize(4);
    fec.setMaxClusterSize(200);
    fec.segment(cluster_indices2);

    Utils::get_clusters_centers(cluster_indices2, cloud_filtered, clusters_centers);
    Utils::filter_clusters(cluster_indices2, cloud_filtered, clusters_centers);

    for (auto center : clusters_centers)
    {
        pcl::PointXYZ cone;
        cone.x = center.x;
        cone.y = center.y;
        cone.z = center.z;
        cones_map->points.push_back(cone);
    }
}


void Perception::ground_align(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::ModelCoefficients::Ptr& coefficients) {

    if (coefficients->values.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "Invalid coefficients size: %zu", coefficients->values.size());
        return;
    }

    Eigen::Vector3f plane_normal(coefficients->values[0],
                                 coefficients->values[1],
                                 coefficients->values[2]);
    plane_normal.normalize();

    Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(plane_normal, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();

    float d = coefficients->values[3];
    Eigen::Vector3f point_on_plane = -d * plane_normal;

    Eigen::Vector3f rotated_point = rotation_matrix * point_on_plane;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = rotation_matrix;
    transform.block<3,1>(0,3) = -rotated_point;

    pcl::transformPointCloud(*cloud, *cloud, transform);
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
    Eigen::Matrix4f T_lidar_to_cog = Eigen::Matrix4f::Identity();
    T_lidar_to_cog(0, 3) = 1.5; // x translation
    pcl::transformPointCloud(*cloud, *cloud, T_lidar_to_cog);

    if (cloud->size() == 0)
    {
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Empty point cloud");
        return;
    }


    GroundFiltering::ransac_ground_filter(cloud,  cloud_filtered,  cloud_plane,  coefficients, 0.05);
    ground_align(cloud, coefficients);
    Cropping::axis_filter<pcl::PointXYZI>(cloud, cloud, "z", 0.08, 1.0);

    if (cloud_buffer_.size() == 0)
    {
        cloud_buffer_.push_back(cloud);
        process_cloud(cloud, prev_cones_);
        return;
    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr acum_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *acum_cloud += *cloud;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = -vx_ * dt; // x translation
    transform(1, 3) = -vy_ * dt; // y translation
    transform(2, 3) = 0.0; // z translation (assuming no vertical movement)
    double phi = - r_ * dt; // rotation around z-axis
    transform(0, 0) = cos(phi);
    transform(0, 1) = -sin(phi);
    transform(1, 0) = sin(phi);
    transform(1, 1) = cos(phi);

    pcl::PointCloud<pcl::PointXYZ>::Ptr actual_cones(new pcl::PointCloud<pcl::PointXYZ>);
    std::map<std::pair<int, int>, double> actual_ground_grid;

    process_cloud(cloud, actual_cones);
    pcl::transformPointCloud(*prev_cones_, *prev_cones_, transform);




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

    // Estimar la transformación con SVD
    Eigen::Matrix4f transform2;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*paired_src, *paired_dst, transform2);


    // pcl::transformPointCloud(*prev_cones, *prev_cones, transform2);
    // *actual_cones += *prev_cones;
    // for (auto point : actual_cones->points)
    // {
    //     PointXYZColorScore p(point.x, point.y, point.z, 1, 0.0f); // color 1 for actual cones
    //     final_map->points.push_back(p);
    // }

    for (auto& pcl : cloud_buffer_)
    {
        pcl::PointCloud<PointXYZColorScore>::Ptr cones_map(new pcl::PointCloud<PointXYZColorScore>);
        pcl::transformPointCloud(*pcl, *pcl, transform);
        pcl::transformPointCloud(*pcl, *pcl, transform2);
        *acum_cloud += *pcl;
    }



    // Publish the map cloud
    // sensor_msgs::msg::PointCloud2 map_msg2;
    // pcl::toROSMsg(*final_map, map_msg2);
    // map_msg2.header.frame_id = "/rslidar";
    // map_pub_->publish(map_msg2);



    if (cloud_buffer_.size() > BUFF_SIZE)
    {
        cloud_buffer_.pop_back();
    }
    std::cout << cloud_filtered->size() << " points in the filtered cloud" << std::endl;
    cloud_buffer_.insert(cloud_buffer_.begin(), cloud);


    // pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    // voxel_filter.setInputCloud(acum_cloud);
    // voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // Adjust leaf size as needed
    // voxel_filter.filter(*acum_cloud);


    // Publish the accumulation cloud
    sensor_msgs::msg::PointCloud2 acum_msg;
    pcl::toROSMsg(*acum_cloud, acum_msg);
    acum_msg.header.frame_id = "/rslidar";
    accumulation_pub_->publish(acum_msg);

    std::cout << "Accumulation time: " << this->now().seconds() - start_time << " seconds" << std::endl;

    // return;

    cloud_filtered->points.clear();
    cloud_plane->points.clear();

    // if (kCrop)
    // {
    //     // Crop the point cloud
    //     double min_x = 0.0;
    //     double min_y = -kMaxYFov;
    //     double min_z = 0.08;
    //     Cropping::box_filter<pcl::PointXYZI>(acum_cloud, acum_cloud, min_x, min_y, min_z, kMaxXFov, kMaxYFov, kMaxZFov);
    //     if (kDebug) RCLCPP_INFO(this->get_logger(), "Cropping Time: %f", this->now().seconds() - start_time);
    // }

    Cropping::axis_filter<pcl::PointXYZI>(acum_cloud, acum_cloud, "z", 0.08, 0.4);


    // Apply the ground filter function
    // GroundFiltering::grid_ground_filter(acum_cloud, cloud_filtered, cloud_plane, coefficients, kThresholdGroundFilter, kMaxXFov, kMaxYFov,
    //     kMaxZFov, kNumberSections, kAngleThreshold);
    // GroundFiltering::pillar_ground_filter(acum_cloud, cloud_filtered, cloud_plane, 0.1, kMaxXFov, kMaxYFov, kNumberSections);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Ground Filter Time: %f", this->now().seconds() - start_time);
    *cloud_filtered = *acum_cloud;

    // Publish the filtered clouds
    // sensor_msgs::msg::PointCloud2 filtered_msg;
    // pcl::toROSMsg(*acum_cloud, filtered_msg);
    // filtered_msg.header.frame_id = "/rslidar";
    // filtered_pub_->publish(filtered_msg);


    // FEC
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);

    FastEuclideanClustering<pcl::PointXYZI> fec;
    fec.setInputCloud(cloud_filtered);
    fec.setSearchMethod(tree);
    fec.setClusterTolerance(0.4);
    fec.setQuality(0.5);
    fec.setMinClusterSize(4);
    fec.setMaxClusterSize(150+20*BUFF_SIZE);
    fec.segment(cluster_indices);

    // Extract the clusters from the point cloud
    // Clustering::euclidean_clustering(cloud_filtered, cluster_indices, kMinClusterSize, kMaxClusterSize);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Clustering time: %f", this->now().seconds() - start_time);


    if (kDebug && cluster_indices.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No clusters found");
        return;
    }


    // Store the clusters centers in a new point cloud
    Utils::get_clusters_centers(cluster_indices, cloud_filtered, clusters_centers);
    if (kDebug) RCLCPP_INFO(this->get_logger(), "Center calculation time: %f", this->now().seconds() - start_time);


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


    if (kDebug && final_map->size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "No cones found");
        return;
    }


    // Estimate the color of the closest cones
    if (kColor)
    {
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
    }


    // Motion correction
    double dt = this->now().seconds() - start_time; // Estimate SDK delay
    Utils::motion_correction(final_map, vx_, vy_, r_, dt);
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