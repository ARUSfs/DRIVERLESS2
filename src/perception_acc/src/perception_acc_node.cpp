/**
 * @file perception_acc_node.hpp
 * @author Álvaro Galisteo Bermúdez (galisbermo03@gmail.com)
 * @brief Main file for the Perception Accumulation node. 
 * Contains the main function and the implementation of the methods to achieve a robust and reliable perception algorithm for the ARUS 
 * Team, which extracts the location of the cones on the track.
 * @version 0.1
 * @date 11-3-2025
 */

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

#include <pcl/filters/voxel_grid.h>

#include <kiss_matcher/FasterPFH.hpp>
#include <kiss_matcher/GncSolver.hpp>
#include <kiss_matcher/KISSMatcher.hpp>
bool DEBUG = false;


std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<PointXYZIRingTime>& cloud) {
    std::vector<Eigen::Vector3f> vec;
    vec.reserve(cloud.size());
    for (const auto& pt : cloud.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      vec.emplace_back(pt.x, pt.y, pt.z);
    }
    return vec;
  }




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
    
    //Create the publishers
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception_acc/points", 10);
    clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/clusters", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/map", 10);

    // Inicializa el tf_buffer y tf_listener de manera que se suscriban al topic /tf
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


}




Eigen::Matrix4f tf2TransformToEigen(const tf2::Transform &tf)
{
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();

    // Rotación
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mat(i, j) = tf.getBasis()[i][j];

    // Traslación
    mat(0, 3) = tf.getOrigin().x();
    mat(1, 3) = tf.getOrigin().y();
    mat(2, 3) = tf.getOrigin().z();

    return mat;
}


/**
 * @brief Create callback function for the lidar topic.
 * @param lidar_msg The point cloud message received from the lidar topic.
 */
void Perception::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{   

    double time = this->get_clock()->now().seconds();
    this->get_tf_position();
    rclcpp::Time t(lidar_msg->header.stamp);

    //Define the variables for the ground filter
    pcl::PointCloud<PointXYZIRingTime>::Ptr cloud(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::fromROSMsg(*lidar_msg, *cloud); 

    pcl::PointCloud<PointXYZIRingTime>::Ptr filtered_cloud(new pcl::PointCloud<PointXYZIRingTime>);

     // Create the filtering object
     pcl::VoxelGrid<PointXYZIRingTime> sor;
     sor.setInputCloud(cloud);
     sor.setLeafSize(0.2f, 0.2f, 0.2f);  // Set the voxel (leaf) size
     sor.filter(*filtered_cloud);

 
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
        std::string frame = "arussim/world";
        std::string world_frame = "slam/vehicle"; 
        tf_msg = tf_buffer_->lookupTransform(frame, world_frame, tf2::TimePoint(std::chrono::nanoseconds(t.nanoseconds())),
                                                tf2::durationFromSec(0.1));

    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    tf2::Transform tf_transform;
    tf2::fromMsg(tf_msg.transform, tf_transform);


    // Convertir a Eigen y aplicar a la nube
    Eigen::Matrix4f transform_matrix;
    transform_matrix = tf2TransformToEigen(tf_transform);


    if (cloud_buffer_.size()==0) {
        cloud_buffer_.push_back(*cloud);
        return;
    } 

    std::cout << "get tf: " << this->get_clock()->now().seconds() - time << std::endl;

    pcl::PointCloud<PointXYZIRingTime>::Ptr updated_filtered_cloud(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::PointCloud<PointXYZIRingTime>::Ptr updated_cloud(new pcl::PointCloud<PointXYZIRingTime>);
    pcl::transformPointCloud(*filtered_cloud, *updated_filtered_cloud, transform_matrix);
    pcl::transformPointCloud(*cloud, *updated_cloud, transform_matrix);

    std::cout << "apply tf: " <<  this->get_clock()->now().seconds() - time << std::endl;
    pcl::PointCloud<PointXYZIRingTime>::Ptr src_pcl(new pcl::PointCloud<PointXYZIRingTime>);

    for (int i = 0; i < std::min(1,int(cloud_buffer_.size())); i++)
    {
        *src_pcl += cloud_buffer_[cloud_buffer_.size()-i-1];
    }

    pcl::VoxelGrid<PointXYZIRingTime> v;
    v.setInputCloud(src_pcl);
    v.setLeafSize(0.2f, 0.2f, 0.2f);  // Set the voxel (leaf) size
    v.filter(*src_pcl);


    auto tgt_pcl = updated_filtered_cloud;





    const float resolution     = 0.3;
    kiss_matcher::KISSMatcherConfig config = kiss_matcher::KISSMatcherConfig(resolution);
    config.use_quatro_ = true;
    config.use_ratio_test_ = false;
    kiss_matcher::KISSMatcher matcher(config);

    const auto& src_vec = convertCloudToVec(*src_pcl);
    const auto& tgt_vec = convertCloudToVec(*tgt_pcl);

    const auto solution = matcher.estimate(src_vec, tgt_vec);
    
    Eigen::Matrix4f solution_eigen      = Eigen::Matrix4f::Identity();
    solution_eigen.block<3, 3>(0, 0)    = solution.rotation.cast<float>();
    solution_eigen.topRightCorner(3, 1) = solution.translation.cast<float>();

    // std::cout << "KISSMatcher solution: " << std::endl;
    // std::cout << solution_eigen << std::endl;
    // matcher.print();
    

    std::cout <<  "KISS: " << this->get_clock()->now().seconds() - time << std::endl;

    //Ensure buffer size limit
    if (cloud_buffer_.size() >= static_cast<size_t>(3)) 
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

    v.setInputCloud(global_cloud);
    v.setLeafSize(0.05f, 0.05f, 0.05f);  // Set the voxel (leaf) size
    v.filter(*global_cloud);



    pcl::transformPointCloud(*global_cloud, *global_cloud, transform_matrix.inverse());



    
// Publish
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*global_cloud, filtered_msg);
    filtered_msg.header.frame_id="/rslidar";
    filtered_pub_->publish(filtered_msg);

    std::cout << "total: " <<  this->get_clock()->now().seconds() - time << std::endl;

    avg_time_ = (N_it_*avg_time_+(this->get_clock()->now().seconds() - time))/(N_it_+1);
    N_it_++;
    std::cout << "avg time: " << avg_time_ << std::endl;

    std::cout << "*********************************************" << std::endl;
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