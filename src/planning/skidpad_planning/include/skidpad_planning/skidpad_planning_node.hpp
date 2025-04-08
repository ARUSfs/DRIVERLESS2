#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>  
#include <visualization_msgs/msg/marker.hpp>
#include<ConeXYZColorScore.h>
#include <common_msgs/msg/trajectory.hpp>
#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility> 
#include <chrono> 
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include <dbscan/dbscan.cpp>

class SkidpadPlanning : public rclcpp::Node {
public:
    SkidpadPlanning();
   
    
private:
    // Node parameters
    std::string kPerceptionTopic;
    std::string kTrajectoryTopic;
    double kPlanningTime;
    double kTargetFirstLap;
    double kTargetSecondLap;
    double kRouteSpacing;
    double kMaxXAccForwards;
    double kMaxXAccBackwards;
    double kMaxYAcc;
    double kStepWidth1;
    double kStepWidth2;
    rclcpp::Time start_time_;
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

    
    
    // Subscription to PointCloud2 messages (Perception)
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;

    // Publisher for planning results (published as visualization markers for the detected lines)
    rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;



    /**
     * @brief Callback function for the perception topic.
     * When the perception topic receives a message, this function is called and performs
     * all the necessary steps to generate the planning.
     * 
     * @param per_msg The point cloud received from the perception.
     */
    void perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg);

    /**
     * @brief Method that processes the point cloud and generates the planning data.
     */ 
    void publish_trajectory();
    void initialize_skidpad(double straight_distance, double circle_radius, 
                                         double first_lap_speed, double second_lap_speed);
    std::tuple<double, double, double> find_circle_center(
       const ConeXYZColorScore& p1, const ConeXYZColorScore& p2, const ConeXYZColorScore& p3);
    // Utility function to convert from ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<ConeXYZColorScore> convert_ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud);
};