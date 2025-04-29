#define PCL_NO_PRECOMPILE

#include "perception_acc/perception_acc_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "external/fast_euclidean_clustering.h"

#include <pcl/filters/voxel_grid.h>

#include <kiss_matcher/FasterPFH.hpp>
#include <kiss_matcher/GncSolver.hpp>
#include <kiss_matcher/KISSMatcher.hpp>
bool DEBUG = true;







Perception::Perception() : Node("perception_acc")
{
    //Declare the parameters
    this->declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    this->declare_parameter<std::string>("state_topic", "/car_state/state");
    this->declare_parameter<double>("voxel_size", 0.3);
    this->declare_parameter<int>("clouds_matched", 1);
    this->declare_parameter<int>("buffer_size", 4);
    this->declare_parameter<bool>("crop", true);
    this->declare_parameter<double>("max_x_fov", 40.0);
    this->declare_parameter<double>("max_y_fov", 40.0);
    this->declare_parameter<double>("max_z_fov", 0.0);
    this->declare_parameter<double>("threshold_ground_filter", 0.05);
    this->declare_parameter<double>("radius", 1.0);
    this->declare_parameter<int>("number_sections", 8);
    this->declare_parameter<double>("angle_threshold", 20);
    this->declare_parameter<int>("minimum_ransac_points", 30);
    this->declare_parameter<double>("threshold_scoring", 0.7);
    this->declare_parameter<double>("distance_threshold", 0.4);
    this->declare_parameter<double>("coloring_threshold", 0.4);

    //Get the parameters
    this->get_parameter("lidar_topic", kLidarTopic);
    this->get_parameter("state_topic", kStateTopic);
    this->get_parameter("voxel_size", kVoxelSize);
    this->get_parameter("clouds_matched", kCloudsMatched);
    this->get_parameter("buffer_size", kBufferSize);
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

    prev_normal_ = std::make_shared<Eigen::Vector3f>(0.0, 0.0, 1.0);

    
    //Create the subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&Perception::lidar_callback, this, std::placeholders::_1));
    
    //Create the publishers
    acum_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception_acc/points", 10);
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception_acc/filtered", 10);
    clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception_acc/clusters", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception_acc/map", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);




}






/**
 * @brief Create callback function for the lidar topic.
 * @param lidar_msg The point cloud message received from the lidar topic.
 */
void Perception::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{   

    double time = this->get_clock()->now().seconds();

    //Define the variables for the ground filter
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::fromROSMsg(*lidar_msg, *cloud); 

    pcl::PointCloud<PointXYZIRingTime>::Ptr filtered_cloud(new pcl::PointCloud<PointXYZIRingTime>);


 
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
        std::string frame = "arussim/world";
        std::string world_frame = "slam/vehicle"; 
        rclcpp::Time t(lidar_msg->header.stamp);
        tf_msg = tf_buffer_->lookupTransform(frame, world_frame, tf2::TimePoint(std::chrono::nanoseconds(t.nanoseconds())),
                                                tf2::durationFromSec(0.1));

    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    tf2::Transform tf_transform;
    tf2::fromMsg(tf_msg.transform, tf_transform);
    Eigen::Matrix4f transform_matrix;
    transform_matrix = tf2TransformToEigen(tf_transform);


    if (cloud_buffer_.size()==0) {
        cloud_buffer_.push_back(*cloud);
        return;
    } 

    pcl::PointCloud<PointXYZIRingTime>::Ptr updated_cloud(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::transformPointCloud(*cloud, *updated_cloud, transform_matrix);

    if (DEBUG) std::cout << "TF transform time: " <<  this->get_clock()->now().seconds() - time << std::endl;
    pcl::PointCloud<PointXYZIRingTime>::Ptr src_pcl(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::PointCloud<PointXYZIRingTime>::Ptr tgt_pcl(new pcl::PointCloud<PointXYZIRingTime>);

    for (int i = 0; i < std::min(kCloudsMatched,int(cloud_buffer_.size())); i++)
    {
        *src_pcl += cloud_buffer_[cloud_buffer_.size()-i-1];
    }


    *tgt_pcl = *updated_cloud;


    Cropping::crop_filter_cropbox(src_pcl, 30.0, 30.0, 0.0);
    Cropping::crop_filter_cropbox(tgt_pcl, 30.0, 30.0, 0.0);


    kiss_matcher::KISSMatcherConfig config = kiss_matcher::KISSMatcherConfig();
    config.voxel_size_ = kVoxelSize;
    config.use_quatro_ = true;
    config.use_ratio_test_ = false;
    config.robin_mode_ = "None";
    kiss_matcher::KISSMatcher matcher(config);

    const auto& src_vec = convertCloudToVec(*src_pcl);
    const auto& tgt_vec = convertCloudToVec(*tgt_pcl);

    const auto solution = matcher.estimate(src_vec, tgt_vec);
    
    Eigen::Matrix4f solution_eigen      = Eigen::Matrix4f::Identity();
    solution_eigen.block<3, 3>(0, 0)    = solution.rotation.cast<float>();
    solution_eigen.topRightCorner(3, 1) = solution.translation.cast<float>();


    

    if (DEBUG) {
        std::cout <<  "KISS-matcher time: " << this->get_clock()->now().seconds() - time << std::endl;
    } 

    //Ensure buffer size limit
    if (cloud_buffer_.size() >= static_cast<size_t>(kBufferSize)) 
    {
        cloud_buffer_.pop_front();
    }
    pcl::PointCloud<PointXYZIRingTime>::Ptr global_cloud(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::PointCloud<PointXYZIRingTime>::Ptr moved_pcl(new pcl::PointCloud<PointXYZIRingTime>);
    std::deque<pcl::PointCloud<PointXYZIRingTime>> new_buffer;
    for (int i = 0; i < cloud_buffer_.size(); i++)
    {
    //     std::cout << "cloud_buffer_ size: " << cloud_buffer_[i].size() << std::endl;
        pcl::transformPointCloud(cloud_buffer_[i], *moved_pcl, solution_eigen);
        *global_cloud += *moved_pcl;
        new_buffer.push_back(*moved_pcl);
    }
    *global_cloud += *updated_cloud;
    new_buffer.push_back(*updated_cloud);
    cloud_buffer_ = new_buffer;

    pcl::VoxelGrid<PointXYZIRingTime> v;
    v.setInputCloud(global_cloud);
    v.setLeafSize(0.1f, 0.1f, 0.01f);  // Set the voxel (leaf) size
    v.filter(*global_cloud);



    pcl::transformPointCloud(*global_cloud, *global_cloud, transform_matrix.inverse());



    
// Publish
    if (DEBUG) {
        sensor_msgs::msg::PointCloud2 acum_msg;
        pcl::toROSMsg(*global_cloud, acum_msg);
        acum_msg.header.frame_id="/rslidar";
        acum_points_pub_->publish(acum_msg);
    }

    if (DEBUG) std::cout << "Accumulation time: " <<  this->get_clock()->now().seconds() - time << std::endl;





    Cropping::crop_filter_cropbox(global_cloud, kMaxXFov, kMaxYFov, kMaxZFov);
    if (DEBUG) std::cout << "Cropping time: " << this->now().seconds() - time << std::endl;


    //Define the variables for the ground filter
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_plane(new pcl::PointCloud<PointXYZIRingTime>);

    //Apply the ground filter fuction
    // GroundFiltering::ransac_ground_filter(global_cloud, cloud_filtered, cloud_plane, prev_normal_, kThresholdGroundFilter);
    GroundFiltering::min_z_ground_filter(global_cloud, cloud_filtered, cloud_plane, kThresholdGroundFilter);
    if (DEBUG) std::cout << "Ground filter time: " << this->now().seconds() - time << std::endl;
    

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
    
    if (DEBUG) std::cout << "Clustering time: " << this->now().seconds() - time << std::endl;


    //Store the clusters centers in a new point cloud
    std::vector<PointXYZColorScore> clusters_centers;
    Perception::get_clusters_centers(cluster_indices, cloud_filtered, clusters_centers);


    //Recover ground points
    Perception::reconstruction(cloud_plane, cloud_filtered, cluster_indices, clusters_centers, kRadius);
    if (DEBUG) std::cout << "Reconstruction time: " << this->now().seconds() - time << std::endl;


    //Filter the clusters by size
    Perception::filter_clusters(cluster_indices, cloud_filtered, clusters_centers);
    if (DEBUG) std::cout << "Cluster Filtering time: " << this->now().seconds() - time << std::endl;


    // Convert the indices of the clusters to the points of the clusters
    std::vector<pcl::PointCloud<PointXYZIRingTime>::Ptr> cluster_points;
    for (const auto& cluster : cluster_indices)
    {
        pcl::PointCloud<PointXYZIRingTime>::Ptr new_cluster(new pcl::PointCloud<PointXYZIRingTime>);
 
        for (const auto& idx : cluster.indices)
        {
            PointXYZIRingTime point = cloud_filtered->points[idx];
            new_cluster->points.push_back(point);
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
    

    //Score the clusters and keep the ones that will be consider cones
    pcl::PointCloud<PointXYZColorScore>::Ptr final_map(new pcl::PointCloud<PointXYZColorScore>);
    Scoring::scoring_surface(final_map, cluster_points, clusters_centers, kThresholdScoring);


    //Print the number of cones and the time of the scoring
    if (DEBUG) std::cout << "Number of cones: " << final_map->size() << std::endl;
    if (DEBUG) std::cout << "Scoring time: " << this->now().seconds() - time << std::endl;


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


    avg_time_ = (N_it_*avg_time_+(this->get_clock()->now().seconds() - time))/(N_it_+1);
    N_it_++;
    std::cout << "Avg time: " << avg_time_ << std::endl;

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
 * @brief Extract the center of each cluster.
 * @param cluster_indices The indices of the points that form each cluster.
 * @param cloud_filtered The input point cloud.
 * @param cluster_centers The center of each cluster.
 */
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



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Perception>());
    rclcpp::shutdown();
    return 0;
}