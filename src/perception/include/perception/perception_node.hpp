/**
 * @file perception_node.hpp
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Header file for the Perception node.
 * Contains the class definition and the declaration of the methods 
 * used in the algorithm. 
 * @version 0.1
 * @date 3-11-2024
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "perception/ground_filtering.h"
#include "perception/clustering.h"
#include "perception/cropping.h"
#include "scoring.h"
#include "PointXYZColorScore.h"
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include "common_msgs/msg/state.hpp"
#include <pcl/common/transforms.h>
#include <deque>
#include <omp.h>
#include <iostream>

/**
 * @class Perception
 * @brief Class containing the Perception node.
 * Manages subscribers and publisher for the different topics used in the algorithm.
 * It also contains the callback functions for the subscribers.
 */
class Perception : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new Perception object.
         * This initializes the perception node, declares parameters, creates
         * the subscribers and publishers and contains all the necessary variables to
         * set up the algorithm.
         */
        Perception();

    private:
        //Parameters
        double kMaxXFov;
        double kMaxYFov;
        double kMaxZFov;
        double kHFov;
        double kThresholdGroundFilter;
        int kNumberSections;
        double kAngleThreshold;
        int kNumberRings;
        int kNumberSectors;
        double kMaxRadius;
        double kRadius;
        int kMinimumRansacPoints;
        double kThresholdScoring;
        double kAccumulationThreshold;
        int kBufferSize;

        // Variables
        double car_x;
        double car_y;
        double car_yaw;
        std::deque<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> cluster_buffer;
        std::deque<std::vector<PointXYZColorScore>> center_buffer;

        //Subscriber
        std::string kLidarTopic;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
        std::string kStateTopic;
        rclcpp::Subscription<common_msgs::msg::State>::SharedPtr state_sub_;

        //Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulation_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

        /**
         * @brief Callback function for the lidar topic.
         * When the lidar topic recieves a message, this function is called and performs
         * all the necessary steps to generate the suitable point cloud.
         * @param lidar_msg The point cloud message received from the lidar.
         */
        void lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg);

        /**
         * @brief Auxiliar function for the call back function.
         * Extract the center of each cluster and keep it in a new point cloud.
         * @param cluster_indices Indices from cluster.
         * @param cloud_filtered The point cloud after the ground filtering.
         * @param cluster_centers The center of each cluster.
         */
        void get_clusters_centers(std::vector<pcl::PointIndices>& cluster_indices,
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& cluster_centers);
            
        /**
         * @brief Auxiliar function for the call back function.
         * Recover points from the cones that were eliminated while ground filtering.
         * @param cloud_plane The input point cloud.
         * @param cloud_filtered The filtered point cloud.
         * @param cluster_indices The indices of the points that form each cluster.
         * @param cluster_centers The center of each cluster.
         * @param radius The radius used to search for eliminated points.
         */
        void reconstruction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, 
            std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore> cluster_centers, 
            double radius);

        /**
        * @brief Auxiliar function for the call back function.
        * @brief Callback function for the car state topic.
        * When the car state topic recieves a message, this function is called and performs
        * all the necessary steps to process the information.
        * @param state_msg The information received from the car state node.
        */
        void state_callback(common_msgs::msg::State::SharedPtr state_msg);

        /**
        * @brief Auxiliar function for the lidar call back function.
        * Accumulates the clusters of the last 5 frames of the lidar call back.
        * @param cluster_points The points of the clusters.
        * @param clusters_centers The centers of the clusters.
        * @param kBufferSize The size of both cluster and center buffers.
        * @param final_clusters THe acumulated clusters.
        * @param final_centers The updated centers.
        */
        void accumulate_clusters(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cluster_points, std::vector<PointXYZColorScore>&
            clusters_centers, int kBufferSize, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& final_clusters, 
            std::vector<PointXYZColorScore>& final_centers, double AccumulationThreshold);
};