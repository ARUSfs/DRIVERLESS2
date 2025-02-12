/**
 * @file accumulation.h
 * @author Álvaro Galisteo Bermúdez (galisbermo03@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to accumulate the point cloud received from the lidar.
 * @version 0.1
 * @date 12-02-2025
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <deque>
#include <iostream>

namespace Accumulation
{
    static std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_buffer;
    static bool buffer_cloud_initialized = false;

    static std::deque<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> cluster_buffer;
    static std::deque<std::vector<PointXYZColorScore>> center_buffer;
    static bool buffer_cluster_initialized = false;

    /**
    * @brief Function to apply rigid transformation to a point cloud
    * @param cloud Points of the cloud.
    * @param vx  Linear velocities along X.
    * @param vy  Linear velocities along Y.
    * @param yaw_rate Yaw rate of the car.
    * @param dt Time interval.
    */
    void rigidTransformation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
            double vx, double vy, double yaw_rate, double dt)
    {
        double delta_x = -vx * dt;
        double delta_y = -vy * dt;
        double delta_theta = -yaw_rate * dt;

        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        transformation(0, 0) = cos(delta_theta);
        transformation(0, 1) = -sin(delta_theta);
        transformation(1, 0) = sin(delta_theta);
        transformation(1, 1) = cos(delta_theta);
        transformation(0, 3) = delta_x;
        transformation(1, 3) = delta_y;

        pcl::transformPointCloud(*cloud, *cloud, transformation);
    }

    /**
    * @brief Accumulate the clouds of the last frames.
    * @param cloud The clouds you want to store in the buffer.
    * @param kBufferSize The size of cloud buffers.
    * @param final_cloud THe acumulated cloud.  
    */
    void accumulate_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int kBufferSize, pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud, 
            double vx, double vy, double yaw_rate, double dt)
    {
        //Clean the buffer the first time
        if (!buffer_cloud_initialized) {
            cloud_buffer.clear();
            buffer_cloud_initialized = true;
        }

        //Ensure buffer size limit
        if (cloud_buffer.size() >= static_cast<size_t>(kBufferSize)) 
        {
            cloud_buffer.pop_front();
        }

        //Add the latest frame
        cloud_buffer.push_back(cloud);

        //Iterate through stored frames (excluding the latest)
        for (size_t i = 0; i < cloud_buffer.size() - 1; ++i) 
        {
            if (!cloud_buffer[i] || cloud_buffer[i]->empty()) 
            {
                continue;
            }
            //Apply the transformation
            rigidTransformation(cloud_buffer[i], vx, vy, yaw_rate, dt);

            //Merge into final accumulated cloud
            *final_cloud += *cloud_buffer[i];
        }

        //Add the latest cloud (unmodified)
        if (!cloud_buffer.back() || cloud_buffer.back()->empty()) 
        {
            return;
        }
        *final_cloud += *cloud_buffer.back();
    }

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
            std::vector<PointXYZColorScore>& final_centers, double AccumulationThreshold,double vx, double vy, double yaw_rate, double dt)
    {
        //Clean the buffer the first time
        if (!buffer_cluster_initialized) {
            cluster_buffer.clear();
            center_buffer.clear();
            buffer_cluster_initialized = true;
        }

        // Remove the oldest cluster if buffer is full
        if (cluster_buffer.size() >= static_cast<size_t>(kBufferSize)) {
            cluster_buffer.pop_front();
            center_buffer.pop_front();
        }
        
        // Add the newest frame 
        cluster_buffer.push_back(cluster_points);
        center_buffer.push_back(clusters_centers);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> all_clusters;
        std::vector<PointXYZColorScore> all_centers;

        // Iterate over all stored frames (except the latest one)
        for (size_t i = 0; i < center_buffer.size() - 1; ++i) {  
            const auto& centers = center_buffer[i];
            auto& clusters = cluster_buffer[i]; 

            for (size_t j = 0; j < centers.size(); ++j) {
                rigidTransformation(clusters[j], vx, vy, yaw_rate, dt);
                all_centers.push_back(centers[j]);
                all_clusters.push_back(clusters[j]);
            }
        }

        // Process the last frame
        const auto& latest_centers = center_buffer.back();
        const auto& latest_clusters = cluster_buffer.back();
        
        for (size_t j = 0; j < latest_centers.size(); ++j) {
            all_centers.push_back(latest_centers[j]);
            all_clusters.push_back(latest_clusters[j]);
        }

        std::vector<bool> merged(all_clusters.size(), false);

        // Merging clusters based on distance threshold
        for (size_t k = 0; k < all_centers.size(); ++k)
        {
            if (merged[k]) continue;
            for (size_t l = 0; l < all_centers.size(); ++l)
            {
                if (k != l)
                {
                    if (merged[l]) continue;

                    double dx = all_centers[k].x - all_centers[l].x;
                    double dy = all_centers[k].y - all_centers[l].y;
                    double squared_distance = dx * dx + dy * dy;

                    if (squared_distance < AccumulationThreshold)
                    {
                        // Add the the points from the comparing cluster to the current cluster
                        for (const auto& point : all_clusters[l]->points)
                        {
                            all_clusters[k]->points.push_back(point);
                        }

                        // Update the coordinates of the current center
                        all_centers[k].x = (all_centers[k].x + all_centers[l].x) / 2;
                        all_centers[k].y = (all_centers[k].y + all_centers[l].y) / 2;
                        all_centers[k].z = (all_centers[k].z + all_centers[l].z) / 2;

                        merged[l] = true;
                    }
                }
            }
        }

        for (size_t n = 0; n < all_centers.size(); ++n)
        {
            if (!merged[n])
            {
                final_clusters.push_back(all_clusters[n]);
                final_centers.push_back(all_centers[n]);
            }
        }
    }
}
