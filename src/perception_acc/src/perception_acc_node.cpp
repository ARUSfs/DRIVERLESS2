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
bool DEBUG = true;

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
    // kHFov *= (M_PI/180);  // Comentada debido a que kHFov no está definido
    
    //Create the subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&Perception::lidar_callback, this, std::placeholders::_1));

    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
        "/car_state/state", 10, std::bind(&Perception::state_callback, this, std::placeholders::_1));
    
    //Create the publishers
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/filtered_cloud", 10);
    clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/clusters", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception_acc/map", 10);

    // Inicializa el tf_buffer y tf_listener de manera que se suscriban al topic /tf
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/**
 * @brief Recover falsely ground filtered points.
 * @param cloud_plane The input point cloud.
 * @param cloud_filtered The filtered point cloud.
 * @param cluster_indices The indices of the points that form each cluster.
 * @param cluster_centers The center of each cluster.
 * @param radius The radius used to search for eliminated points.
 */
void Perception::reconstruction(pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_plane, pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered, 
    std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore> clusters_centers, 
    double radius)
{
    //Iterate on clusters
    for (size_t i = 0; i < clusters_centers.size(); ++i)
    {
        //Convert from PointXYZColorScore to PointXYZI
        PointXYZIRingTime center;
        center.x = clusters_centers[i].x;
        center.y = clusters_centers[i].y;
        center.z = clusters_centers[i].z;
        center.intensity = clusters_centers[i].score;
        
        //Iterate on planar points
        for (size_t j = 0; j < cloud_plane->size(); ++j)
        {
            PointXYZIRingTime point = cloud_plane->points[j];

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
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& clusters_centers)
{
    for (int i = cluster_indices.size() - 1; i >= 0; i--)
    {
        //Create a temporal point cloud
        pcl::PointIndices indices = cluster_indices[i];
        pcl::PointCloud<PointXYZIRingTime>::Ptr cluster_cloud(new pcl::PointCloud<PointXYZIRingTime>);
        pcl::copyPointCloud(*cloud_filtered, indices, *cluster_cloud);

        //Obtain the new bounding box of the cluster
        PointXYZIRingTime min_point, max_point;
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

void Perception::get_clusters_centers(std::vector<pcl::PointIndices>& cluster_indices,
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& clusters_centers)
{
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); )
    {
        //Create a temporal point cloud
        pcl::PointCloud<PointXYZIRingTime>::Ptr cluster_cloud(new pcl::PointCloud<PointXYZIRingTime>);
        pcl::copyPointCloud(*cloud_filtered, *it, *cluster_cloud);

        //Obtain the new bounding box of the cluster
        PointXYZIRingTime min_point, max_point;
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
 * @brief Callback function for the car state topic.
 * @param state_msg The state message received from the car state topic.
 */
void Perception::state_callback(const common_msgs::msg::State::SharedPtr state_msg)
{
    // Get the velocity of the car
    double vx = state_msg->vx;
    double vy = state_msg->vy;
    double yaw_rate = state_msg->r;
}

/**
 * @brief Create callback function for the lidar topic.
 * @param lidar_msg The point cloud message received from the lidar topic.
 */
void Perception::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{   
    double time_start = this->now().seconds();
    this->get_tf_position();

    //Define the variables for the ground filter
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::fromROSMsg(*lidar_msg, *cloud);

    // Accumulation
    pcl::PointCloud<PointXYZIRingTime>::Ptr accumulated_cloud(new pcl::PointCloud<PointXYZIRingTime>);
    Accumulation::accumulate(cloud, accumulated_cloud, x_, y_, yaw_, kDistanceLidarToCoG);
        
    RCLCPP_INFO(this->get_logger(), "Accumulated cloud size: %zu", accumulated_cloud->size());

    // Skip if empty
    if (accumulated_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Accumulated cloud is empty. Skipping ground filter.");
        return;
    }

    // Cropping
    Cropping::crop_filter_cropbox(accumulated_cloud, kMaxXFov, kMaxYFov, kMaxZFov);
    if (DEBUG) std::cout << "Cropping Time: " << this->now().seconds() - time_start << std::endl;


    //Define the variables for the ground filter
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_plane(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);


    //Apply the ground filter fuction
    GroundFiltering::grid_ground_filter(accumulated_cloud, cloud_filtered, cloud_plane, coefficients,
        kThresholdGroundFilter, kMaxXFov, kMaxYFov, kMaxZFov, kNumberSections, kAngleThreshold, kMinimumRansacPoints);
    if (DEBUG) std::cout << "Ground Filter Time: " << this->now().seconds() - time_start << std::endl;
    

    //Extract the clusters from the point cloud
    // std::vector<pcl::PointIndices> cluster_indices;
    // Clustering::euclidean_clustering(cloud_filtered, cluster_indices);


    // FAST EUCLIDEAN CLUSTERING
    pcl::search::KdTree<PointXYZIRingTime>::Ptr tree(new pcl::search::KdTree<PointXYZIRingTime>);
    std::vector<pcl::PointIndices> cluster_indices;

    FastEuclideanClustering<PointXYZIRingTime> fec;
    fec.setInputCloud(cloud_filtered);
    fec.setSearchMethod(tree);
    fec.setClusterTolerance(0.5);
    fec.setQuality(0.5);
    fec.setMinClusterSize(4);     
    fec.setMaxClusterSize(200);

    fec.segment(cluster_indices);
    
    if (DEBUG) std::cout << "Clustering time: " << this->now().seconds() - time_start << std::endl;


    //Store clusters centers
    std::vector<PointXYZColorScore> clusters_centers;
    this->get_clusters_centers(cluster_indices, cloud_filtered, clusters_centers);
    if (DEBUG) std::cout << "Number of posibles cones: " << clusters_centers.size() << std::endl;


    //Recover ground points
    Perception::reconstruction(cloud_plane, cloud_filtered, cluster_indices, clusters_centers, kRadius);
    if (DEBUG) std::cout << "Reconstruction time: " << this->now().seconds() - time_start << std::endl;


    //Filter clusters by size
    Perception::filter_clusters(cluster_indices, cloud_filtered, clusters_centers);
    if (DEBUG) std::cout << "Filtering time: " << this->now().seconds() - time_start << std::endl;


    // Convert indices to cluster point clouds
    std::vector<pcl::PointCloud<PointXYZIRingTime>::Ptr> cluster_points;
    for (const auto& cluster : cluster_indices)
    {
        pcl::PointCloud<PointXYZIRingTime>::Ptr new_cluster(new pcl::PointCloud<PointXYZIRingTime>);
        for (const auto& idx : cluster.indices)
        {
            new_cluster->points.push_back(cloud_filtered->points[idx]);
        } 
        cluster_points.push_back(new_cluster);
    }


    pcl::PointCloud<PointXYZIRingTime>::Ptr clusters_cloud(new pcl::PointCloud<PointXYZIRingTime>);
    if (DEBUG) {
        // Publish clusters cloud with different colors
        int i = 0;
        for (auto c : cluster_points)
        {
            for (auto &p : c->points)
            {
                PointXYZIRingTime q = p;
                q.intensity = i;
                clusters_cloud->push_back(q);
            }
            i++;
        }
    }
    

    // Convert cluster_points to type pcl::PointXYZI para scoring
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_points_i;
    for (auto &cluster : cluster_points) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_i(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto &p : cluster->points) {
            pcl::PointXYZI pt;
            pt.x = p.x; pt.y = p.y; pt.z = p.z; pt.intensity = p.intensity;
            cluster_i->push_back(pt);
        }
        cluster_points_i.push_back(cluster_i);
    }

    pcl::PointCloud<PointXYZColorScore>::Ptr final_map(new pcl::PointCloud<PointXYZColorScore>);
    Scoring::scoring_surface(final_map, cluster_points_i, clusters_centers, kThresholdScoring);
    

    //Print the number of cones and the time of the scoring
    if (DEBUG) std::cout << "Number of cones: " << final_map->size() << std::endl;
    if (DEBUG) std::cout << "Scoring time: " << this->now().seconds() - time_start << std::endl;


    //Estime the color of the closest cones
    // ColorEstimation::color_estimation(cluster_points, clusters_centers, kDistanceThreshold, kColoringThreshold);
    // if (DEBUG) std::cout << "Color estimation time: " << this->now().seconds() - time << std::endl;


    // //Update the colors of final map points
    // for (auto& point : final_map->points) 
    // {
    //     for (const auto& center : clusters_centers) 
    //     {
    //         if (point.x == center.x && point.y == center.y && point.z == center.z) 
    //         {
    //             point.color = center.color;
    //         }
    //     }
    // }

    // // Motion correction
    // double dt = this->now().seconds() - time; // Estimate sdk delay
    // double theta = -yaw_rate*(dt);
    // for (auto& p : final_map->points)
    // {
    //     // double point_delay = std::atan(p.y/p.x)/(2*M_PI)*0.1;
    //     double dx = vx*dt, dy = vy*dt;
    //     p.x = p.x*std::cos(theta) - p.y*std::sin(theta) - dx;
    //     p.y = p.x*std::sin(theta) + p.y*std::cos(theta) - dy;
    // }

    if (DEBUG) std::cout << "//////////////////////////////////////////////" << std::endl;

    if (DEBUG){
        //Publish the filtered cloud
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_filtered, filtered_msg);
        filtered_msg.header.frame_id="/rslidar";
        filtered_pub_->publish(filtered_msg);

        // Publish the clusters cloud
        sensor_msgs::msg::PointCloud2 clusters_msg;
        pcl::toROSMsg(*clusters_cloud, clusters_msg);
        clusters_msg.header.frame_id="/rslidar";
        clusters_pub_->publish(clusters_msg);
    }

    //Publish the map cloud
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*final_map, map_msg);
    map_msg.header.frame_id="/rslidar";
    map_pub_->publish(map_msg);

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
        transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
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
        
        // RCLCPP_INFO(this->get_logger(), "Transform: x: %f, y: %f, yaw: %f", x_, y_, yaw_);
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