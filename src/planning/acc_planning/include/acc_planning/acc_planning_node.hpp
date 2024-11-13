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
class AccPlanning : public rclcpp::Node {
public:
    AccPlanning();

private:
    // Node parameters
    std::string kPerceptionTopic;

    pcl::PointCloud<ConeXYZColorScore> cones_;
    common_msgs::msg::Trajectory msg;

    double a_;
    double b_;

    
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
     * 
     * @param per_msg The point cloud message received in the callback.
     * @return common_msgs::msg::ConeXYZColorScore Object containing the processed planning results.
     */ 
    void generate_planning();
    void publish_trajectory();

    // Utility function to convert from ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<ConeXYZColorScore> convert_ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud);
};