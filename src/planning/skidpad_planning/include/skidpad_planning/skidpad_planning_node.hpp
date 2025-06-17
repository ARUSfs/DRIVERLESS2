/**
 * @file skidpad_planning_node.cpp
 * @author David Guil (davidguilb2@gmail.com)
 * @brief Skidpad planning node header for the ARUS Driverless pipeline.
 */

 #include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>  
#include<ConeXYZColorScore.h>
#include <common_msgs/msg/trajectory.hpp>
#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
#include <chrono>
#include <Eigen/Dense>

#include <dbscan/dbscan.cpp>

class SkidpadPlanning : public rclcpp::Node {
public:
    SkidpadPlanning();

private:    
    // Variables
    rclcpp::Time start_time_;
    rclcpp::Time start_time_2;
    double n1;
    double n2;
    std::pair<double, double> left_center;
    std::pair<double, double> right_center;
    std::vector<Point> centers;
    double radius = 9.125;
    bool trajectory_calculated_ = false;
    std::vector<Eigen::Vector2d> template_;
    std::vector<double> s_;
    std::vector<double> k_;
    std::vector<double> speed_profile_;
    std::vector<double> acc_profile_;
    pcl::PointCloud<ConeXYZColorScore> cones_;
    common_msgs::msg::Trajectory msg;

    rclcpp::TimerBase::SharedPtr timer_;
    int ransac_counter = 0;

    //nuevas
    double a_;
    double b_;
    std::vector<double> a_history_;
    std::vector<double> b_history_;
    double sum_a_ = 0.0;
    double sum_b_ = 0.0;
    const size_t history_size_ = 1;
    //

    // Parameters
    std::string kPerceptionTopic;       // Topic for receiving perception data
    std::string kTrajectoryTopic;       // Topic for publishing the trajectory
    double kPlanningTime;               // Time for computing the skidpad centers
    double kTargetFirstLap;             // Target time for the first lap
    double kTargetSecondLap;            // Target time for the second lap
    double kRouteSpacing;               // Spacing between trajectory points
    double kMaxXAccForwards;            // Maximum forward acceleration (m/s^2)
    double kMaxXAccBackwards;           // Maximum backward acceleration (m/s^2)
    double kMaxYAcc;                    // Maximum lateral acceleration (m/s^2)
    double kStepWidth1;                 // Step width for trajectory generation (phase 1)
    double kStepWidth2;                 // Step width for trajectory generation (phase 2)
    bool kDebug;                        // Debug mode (true by default)


    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;

    // Publishers
    rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

    /**
     * @brief Callback function for processing the perception data.
     */
    void perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg);

   
    /**
     * @brief Publishes the computed skidpad trajectory to the corresponding topic.
     */
    void publish_trajectory();

    /**
     * @brief Initializes the skidpad trajectory template and speed/acceleration profiles.
     */
    void initialize_skidpad(double straight_distance, double circle_radius, 
                            double first_lap_speed, double second_lap_speed);

    /**
     * @brief Finds the center and radius of a circle given three points.
     */
    std::tuple<double, double, double> find_circle_center(
        const ConeXYZColorScore& p1, const ConeXYZColorScore& p2, const ConeXYZColorScore& p3);

    /**
     * @brief Converts a ROS PointCloud2 message to a PCL point cloud.
     */

    void compute_ransac_centers();

    //borrar
    void publish_aux_trajectory();

    void calculate_aux_profiles();
    
    pcl::PointCloud<ConeXYZColorScore> convert_ros_to_pcl(
        const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud);

        double check_trajectory();
};