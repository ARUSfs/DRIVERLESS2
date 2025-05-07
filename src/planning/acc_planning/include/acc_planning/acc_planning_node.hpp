#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>  
#include <common_msgs/msg/trajectory.hpp>
#include <ConeXYZColorScore.h>
#include <iostream>
#include <random>

/**
 * @class AccPlanning
 * @brief Node for autonomous driving trajectory planning based on perception data.
 */
class AccPlanning : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the AccPlanning node.
     */
    AccPlanning();

private:
    // Parameters
    std::string kPerceptionTopic;
    std::string kTrajectoryTopic;
    double kTargetSpeed;
    double kMaxXAcc;
    double kMaxDec;
    double kTrackLength;
    bool kDebug ;

    // Data structures
    pcl::PointCloud<ConeXYZColorScore> cones_;
    std::vector<double> s_;
    std::vector<double> speed_profile_;
    std::vector<double> acc_profile_;

    // Line coefficients for trajectory
    double a_;
    double b_;
    std::vector<double> a_history_;
    std::vector<double> b_history_;
    double sum_a_ = 0.0;
    double sum_b_ = 0.0;
    const size_t history_size_ = 1;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;
    rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

    /**
     * @brief Callback function for processing perception data.
     * @param per_msg The point cloud received from the perception topic.
     */
    void perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg);

    /**
     * @brief Generates the trajectory planning based on the detected cones.
     */
    void generate_planning();

    /**
     * @brief Calculates the speed and acceleration profiles for the trajectory.
     */
    void calculate_profiles();

    /**
     * @brief Publishes the generated trajectory to the trajectory topic.
     */
    void publish_trajectory();

    /**
     * @brief Converts a ROS PointCloud2 message to a PCL PointCloud.
     * @param ros_cloud The input ROS PointCloud2 message.
     * @return The converted PCL PointCloud.
     */
    pcl::PointCloud<ConeXYZColorScore> convert_ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud);
};