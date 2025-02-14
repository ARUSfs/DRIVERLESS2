/**
 * @file perception_node.hpp
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com) and Álvaro Galisteo (galisbermo03@gmail.com)
 * @brief Main file for the Perception node. 
 * Contains the main function and the implementation of the methods to achieve a robust and reliable perception algorithm for the ARUS 
 * Team, which extracts the location of the cones on the track.
 * @version 0.2
 * @date 05-02-2025
 */

#include "perception/perception_node.hpp"
bool DEBUG = true;

Perception::Perception() : Node("Perception")
{
    //Declare the parameters
    this->declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    this->declare_parameter<std::string>("state_topic", "/car_state/state");
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
    this->declare_parameter<double>("threshold_scoring", 0.8);
    this->declare_parameter<double>("distance_threshold", 0.4);
    this->declare_parameter<double>("coloring_threshold", 0.4);
    this->declare_parameter<double>("accumulation_threshold", 0.01);
    this->declare_parameter<int>("buffer_size", 10);
    this->declare_parameter<bool>("accumulation_clouds", false);
    this->declare_parameter<bool>("accumulation_clusters", false);

    //Get the parameters
    this->get_parameter("lidar_topic", kLidarTopic);
    this->get_parameter("state_topic", kStateTopic);
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
    this->get_parameter("distance_threshold", kDistanceThreshold);
    this->get_parameter("coloring_threshold", kColoringThreshold);
    this->get_parameter("accumulation_threshold", kAccumulationThreshold);
    this->get_parameter("buffer_size", kBufferSize);
    this->get_parameter("accumulation_clouds", kAccumulation_clouds);
    this->get_parameter("accumulation_clusters", kAccumulation_clusters);

    // Initialize the variables
    vx = 0.0;
    vy = 0.0;
    yaw_rate = 0.0;
    dt = 0.1;

    //Transform into radians
    kHFov *= (M_PI/180);
    
    //Create the subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&Perception::lidar_callback, this, std::placeholders::_1));
    if (kAccumulation_clouds || kAccumulation_clusters)
    {
        state_sub_ = this->create_subscription<common_msgs::msg::State>(
            kStateTopic, 10, std::bind(&Perception::state_callback, this, std::placeholders::_1));
    }
    
    //Create the publishers
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/filtered_cloud", 10);
    clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/clusters", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/map", 10);
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
        if ((max_z - min_z) < 0.4 && (max_x - min_x) < 0.4 && (max_y - min_y) < 0.4)
        {
            PointXYZColorScore center;
            center.x = (max_x + min_x) / 2;
            center.y = (max_y + min_y) / 2;
            center.z = (max_z + min_z) / 2;
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

void Perception::get_clusters_centers_ransac(std::vector<pcl::PointIndices>& cluster_indices,
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

        if ((max_z - min_z) < 0.4 && (max_x - min_x) < 0.4 && (max_y - min_y) < 0.4)
        {
            PointXYZColorScore center;
            pcl::PointCloud<pcl::PointXYZ>::Ptr base_points(new pcl::PointCloud<pcl::PointXYZ>);

            for (const auto& index : it->indices)
            {
                const auto& point = cloud_filtered->points[index];

                if (std::abs(point.z - min_z) < 2.0) 
                {
                    base_points->push_back(pcl::PointXYZ(point.x, point.y, 0));
                }
            }

            if (base_points->size() > 6)
            {
                //std::cout << "Base points: " << base_points->size() << std::endl;

                center.z = (max_z + min_z) / 2;

                //std::cout << "Z: " << center.z << std::endl;

                pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr model_circle(
                    new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(base_points));

                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle);
                ransac.setDistanceThreshold(1.0);
                ransac.computeModel();

                Eigen::VectorXf circle_coefficients;
                ransac.getModelCoefficients(circle_coefficients);

                center.x = circle_coefficients[0]; 
                center.y = circle_coefficients[1]; 

                //std::cout << "X: " << center.x << std::endl;
                //std::cout << "Y: " << center.y << std::endl;

                center.color = 0;
                center.score = 1;
                clusters_centers.push_back(center);

                //std::cout << "Clusters centers: " << clusters_centers.size() << std::endl;

                it++;
            }
            else
            {
                center.x = (max_x + min_x) / 2;
                center.y = (max_y + min_y) / 2;
                center.z = (max_z + min_z) / 2;
                center.color = 0;
                center.score = 0;
                clusters_centers.push_back(center);

                it++;
            }
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
        if ((max_z - min_z) < 0.1 || (max_z - min_z) > 0.4 || (max_x - min_x) > 0.4 || (max_y - min_y) > 0.4)
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
* @brief Create callback function for the car state topic.
* @param state_msg The information received from the car state node.
*/
void Perception::state_callback(const common_msgs::msg::State::SharedPtr state_msg)
{
    vx = state_msg->vx;
    vy = state_msg->vy;
    yaw_rate = state_msg->r;
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
    if (DEBUG) std::cout << "Cropping Time: " << this->now().seconds() - start_time << std::endl;

    //Define the variables for the ground filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    //Apply the ground filter fuction
    GroundFiltering::grid_ground_filter(cloud, cloud_filtered, cloud_plane, coefficients, kThresholdGroundFilter, kMaxXFov, kMaxYFov, kMaxZFov, kNumberSections, kAngleThreshold, kMinimumRansacPoints);
    
    //Print the time of the ground filter algorithm used
    if (DEBUG) std::cout << "Ground Filter Time: " << this->now().seconds() - start_time << std::endl;

    if (kAccumulation_clouds)
    {
        //Accumulate the filtered clouds
        cloud_filtered = Accumulation::accumulate_cloud(cloud_filtered, kBufferSize, vx, vy, yaw_rate, dt);
    }
    
    //Extract the clusters from the point cloud
    std::vector<pcl::PointIndices> cluster_indices;
    Clustering::euclidean_clustering(cloud_filtered, cluster_indices);

    //Print the time of the clustering function
    if (DEBUG) std::cout << "Clustering time: " << this->now().seconds() - start_time << std::endl;

    //Store the clusters centers in a new point cloud
    std::vector<PointXYZColorScore> clusters_centers;
    Perception::get_clusters_centers_ransac(cluster_indices, cloud_filtered, clusters_centers);

    /*for (int i = 0; i < clusters_centers.size(); ++i)
    {
        std::cout << "1-" << clusters_centers[i].x <<  clusters_centers[i].y << clusters_centers[i].z << std::endl;
        std::cout << "2-" << clusters_centers2[i].x <<  clusters_centers2[i].y << clusters_centers2[i].z << std::endl;
    }*/

    //Print the number of possibles cones
    if (DEBUG) std::cout << "Number of posibles cones: " << clusters_centers.size() << std::endl;

    //Recover ground points
    Perception::reconstruction(cloud_plane, cloud_filtered, cluster_indices, clusters_centers, kRadius);
    
    //Print the time of the reconstruction function
    if (DEBUG) std::cout << "Reconstruction time: " << this->now().seconds() - start_time << std::endl;

    //Filter the clusters by size
    Perception::filter_clusters(cluster_indices, cloud_filtered, clusters_centers);

    //Print the time of the filtering function
    if (DEBUG) std::cout << "Filtering time: " << this->now().seconds() - start_time << std::endl;

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

    if (kAccumulation_clusters)
    {
        // Accumulate the clusters
        cluster_points = Accumulation::accumulate_clusters(cluster_points, clusters_centers, kBufferSize, kAccumulationThreshold, vx, vy, yaw_rate, dt);
    }

    // Merge clusters into a single point cloud
    int i = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr clusters_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto c : cluster_points)
    {
        for (auto &p : c->points)
        {
            p.intensity = i;
            clusters_cloud->push_back(p);
        }
        i++;
    }
    if (DEBUG) std::cout << "Accumulation time: " << this->now().seconds() - start_time << std::endl;


    //Score the clusters and keep the ones that will be consider cones
    pcl::PointCloud<PointXYZColorScore>::Ptr final_map(new pcl::PointCloud<PointXYZColorScore>);
    Scoring::scoring_surface(cloud_filtered, final_map, cluster_indices, clusters_centers, kThresholdScoring);

    //Print the number of cones and the time of the scoring
    if (DEBUG) std::cout << "Number of cones: " << final_map->size() << std::endl;
    if (DEBUG) std::cout << "Scoring time: " << this->now().seconds() - start_time << std::endl;

    //Estime the color of the closest cones
    ColorEstimation::color_estimation(cluster_indices, clusters_centers, cloud_filtered, kDistanceThreshold, kColoringThreshold);

    //Print the time of the color estimation function
    if (DEBUG) std::cout << "Color estimation time: " << this->now().seconds() - start_time << std::endl;

    //Update the colors of final map points
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Perception>());
    rclcpp::shutdown();
    return 0;
}