/**
 * @file acc_planning_mode.hpp
 * @author David Guil (davidguilb2@gmail.com)
 * @brief Acceleration planning node header for the ARUS Driverless pipeline.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>  
#include <common_msgs/msg/trajectory.hpp>
#include <ConeXYZColorScore.h>
#include <iostream>
#include <random>



class AccPlanning : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the AccPlanning node.
     */
    AccPlanning();

private:

    // Variables
    pcl::PointCloud<ConeXYZColorScore> cones_;
    std::vector<double> s_;
    std::vector<double> speed_profile_;
    std::vector<double> acc_profile_;

    double a_;
    double b_;

    // Parameters
    std::string kPerceptionTopic;
    std::string kTrajectoryTopic;
    double kTargetSpeed;
    double kMaxXAcc;
    double kMaxDec;
    double kTrackLength;
    bool kDebug;
    double kStep;
    int kMaxIterations;
    double kRansacThreshold;
    int kMinInliersRequired;
    double kMaxAllowedDeviation;


    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;

    // Publishers
    rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;


    /**
     * @brief Callback function for processing perception data.
     */
    void perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg);

    /**
     * @brief Generates the trajectory planning based on the detected cones.
     */
    void generate_planning();

    /**
     * @brief Calculates the speed and acceleration profiles for the trajectory.
     */
    void compute_profiles();

    /**
     * @brief Publishes the generated trajectory to the trajectory topic.
     */
    void publish_trajectory();

    /**
     * @brief Converts a ROS PointCloud2 message to a PCL PointCloud.
     */
    pcl::PointCloud<ConeXYZColorScore> convert_ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud);
};