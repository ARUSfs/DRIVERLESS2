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
#include <pcl/registration/icp.h>
#include <deque>
#include <iostream>
#include "PointXYZIRingTime.h"
#include <pcl/filters/voxel_grid.h> 
#include "perception_acc/clustering.h"
#include <chrono>

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>


namespace Accumulation
{
    static std::deque<pcl::PointCloud<PointXYZIRingTime>::Ptr> cloud_buffer;
    static pcl::PointCloud<PointXYZIRingTime>::Ptr global_cloud(new pcl::PointCloud<PointXYZIRingTime>);
    static bool buffer_cloud_initialized = false;

    static std::deque<std::vector<pcl::PointCloud<PointXYZIRingTime>::Ptr>> cluster_buffer;
    static std::deque<std::vector<PointXYZColorScore>> center_buffer;
    static bool buffer_cluster_initialized = false;

    double delta_x;
    double delta_y;
    double delta_theta;

    /**
    * @brief Function to apply rigid transformation to a point cloud
    * @param cloud Points of the cloud.
    * @param vx  Linear velocities along X.
    * @param vy  Linear velocities along Y.
    * @param yaw_rate Yaw rate of the car.
    * @param dt Time interval.
    */
    void rigidTransformation(pcl::PointCloud<PointXYZIRingTime>::Ptr cloud, 
            double vx, double vy, double yaw_rate, double dt)
    {
        // Compute translation and rotation
        double delta_x = -vx * dt;
        double delta_y = -vy * dt;
        double delta_theta = -yaw_rate * dt;

        // Transformation from LIDAR to CoG
        Eigen::Affine3f transform_lidar_to_CoG = Eigen::Affine3f::Identity();
        transform_lidar_to_CoG.translation() << -1.5, 0.0, 0.0;

        // Motion transformation matrix (rotation + translation)
        Eigen::Affine3f transform_motion = Eigen::Affine3f::Identity();
        transform_motion.linear() << cos(delta_theta), -sin(delta_theta), 0,
                                    sin(delta_theta), cos(delta_theta),  0,
                                    0,               0,                1;
        transform_motion.translation() << delta_x, delta_y, 0.0;

        // Transformation from CoG back to LIDAR
        Eigen::Affine3f transform_CoG_to_lidar = Eigen::Affine3f::Identity();
        transform_CoG_to_lidar.translation() << 1.5, 0.0, 0.0;

        // Compute final transformation matrix
        Eigen::Affine3f final_transform = transform_lidar_to_CoG * transform_motion * transform_CoG_to_lidar;

        // Apply transformation to the **point cloud**
        pcl::transformPointCloud(*cloud, *cloud, final_transform.matrix());
    }



    Eigen::Isometry3d compensator(std::vector<PointXYZColorScore> global_center_clusters, std::vector<PointXYZColorScore> center_clusters)
    {
        if(global_center_clusters.empty() || center_clusters.empty()){
            return Eigen::Isometry3d::Identity();
        }
        
        std::vector<std::tuple<PointXYZColorScore, PointXYZColorScore>> near_points;
        
        for (int i = 0; i < center_clusters.size(); i++) {
            double min_dist = 0.2;
            bool exists = false;
            PointXYZColorScore nearest_global;
            for (int j = 0; j < global_center_clusters.size(); j++) {
                double distance = sqrt(pow(center_clusters[i].x - global_center_clusters[j].x, 2) +
                                       pow(center_clusters[i].y - global_center_clusters[j].y, 2));
                if (distance < min_dist) {
                    exists = true;
                    nearest_global = global_center_clusters[j];
                    min_dist = distance;
                }
            }
            if(exists) near_points.emplace_back(std::make_tuple(nearest_global, center_clusters[i]));
        }
        
        if(near_points.empty()){
            RCLCPP_WARN(rclcpp::get_logger("Accumulation"), 
                        "No matching clusters found for compensation. Returning identity transformation.");
            return Eigen::Isometry3d::Identity();
        }
        
        double avg_x = 0.0;
        double avg_y = 0.0;
        for (int i = 0; i < near_points.size(); i++) {
            PointXYZColorScore global_point = std::get<0>(near_points[i]);
            PointXYZColorScore local_point = std::get<1>(near_points[i]);
            avg_x += global_point.x - local_point.x;
            avg_y += global_point.y - local_point.y;
        }
        
        avg_x /= near_points.size();
        avg_y /= near_points.size();
        
        Eigen::Isometry3d compensation = Eigen::Isometry3d::Identity();
        compensation.translation() << avg_x, avg_y, 0.0;
        compensation.linear() << 1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0;        
        RCLCPP_INFO(rclcpp::get_logger("Accumulation"),
            "Compensation (vector): [%f, %f, %f]",
            compensation.translation()[0],
            compensation.translation()[1],
            compensation.translation()[2]);
        
        return compensation;
    }



    /**
     * @brief Computes the rigid transformation matrix based on the given parameters.
     * @param x Translation along X-axis
     * @param y Translation along Y-axis
     * @param yaw Rotation around Z-axis (yaw)
     * @param kDistanceLidarToCoG Distance from LIDAR to CoG
     * @return Rigid transformation matrix
     */
    Eigen::Isometry3d computeRigidTransformation(double x, double y, double yaw, double kDistanceLidarToCoG) {
        // Transformation from LIDAR to CoG
        Eigen::Isometry3d transform_lidar_to_CoG = Eigen::Isometry3d::Identity();
        transform_lidar_to_CoG.translation() << -kDistanceLidarToCoG, 0.0, 0.0;

        // Motion transformation matrix (rotation + translation)
        Eigen::Isometry3d transform_motion = Eigen::Isometry3d::Identity();
        transform_motion.linear() << cos(yaw), -sin(yaw), 0,
                                      sin(yaw),  cos(yaw), 0,
                                      0,         0,        1;
        transform_motion.translation() << x, y, 0.0;

        // Transformation from CoG back to LIDAR
        Eigen::Isometry3d transform_CoG_to_lidar = Eigen::Isometry3d::Identity();
        transform_CoG_to_lidar.translation() << kDistanceLidarToCoG, 0.0, 0.0;

        // Compute final transformation matrix
        Eigen::Isometry3d final_transform = transform_lidar_to_CoG * transform_motion * transform_CoG_to_lidar;

        return final_transform;  // Return the transformation matrix
    }



    pcl::PointCloud<PointXYZIRingTime>::Ptr accumulate_global_cloud_ring(
        pcl::PointCloud<PointXYZIRingTime>::Ptr cloud, std::vector<PointXYZColorScore> clusters_centers,
        double x, double y, double yaw, double kDistanceLidarToCoG, float kDownsampleSize)
    {
        auto final_transform = computeRigidTransformation(x, y, yaw, kDistanceLidarToCoG);

        pcl::PointCloud<PointXYZIRingTime>::Ptr transformed_cloud(new pcl::PointCloud<PointXYZIRingTime>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, final_transform.matrix());

        // Apply voxel grid filter to limit the size of the global cloud
        static pcl::VoxelGrid<PointXYZIRingTime> global_vg; 
        global_vg.setInputCloud(global_cloud);
        global_vg.setLeafSize(kDownsampleSize, kDownsampleSize, kDownsampleSize/10);
        pcl::PointCloud<PointXYZIRingTime>::Ptr filtered(new pcl::PointCloud<PointXYZIRingTime>());
        global_vg.filter(*filtered);

        // Limit the size of the global cloud by replacing it with the filtered cloud
        global_cloud = filtered;

        // Accumulate transformed cloud
        *global_cloud += *transformed_cloud;

        return global_cloud;
    }



    pcl::PointCloud<PointXYZIRingTime>::Ptr accumulate_local_cloud_ring(
        pcl::PointCloud<PointXYZIRingTime>::Ptr cloud, std::vector<PointXYZColorScore> clusters_centers,
        double x, double y, double yaw, double kDistanceLidarToCoG, float kDownsampleSize)
    {
        //Clean the buffer the first time
        if (!buffer_cloud_initialized) {
            cloud_buffer.clear();
            buffer_cloud_initialized = true;
            cloud_buffer.push_back(cloud);
            return cloud;
        }

        if (!cloud || cloud->empty()) {
            return cloud; // or create and return an empty cloud
        }
    
        auto final_transform = computeRigidTransformation(x, y, yaw, kDistanceLidarToCoG);

        //Ensure buffer size limit
        if (cloud_buffer.size() >= static_cast<size_t>(20)) 
        {
            cloud_buffer.pop_front();
        }

        pcl::PointCloud<PointXYZIRingTime>::Ptr global_cloud(new pcl::PointCloud<PointXYZIRingTime>);
        for (int i = 0; i < cloud_buffer.size(); i++)
        {
            *global_cloud += *cloud_buffer[i];
        }
            // IMPLEMENTATION SMALL_GICP
            pcl::PointCloud<pcl::PointCovariance>::Ptr target = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<PointXYZIRingTime>, pcl::PointCloud<pcl::PointCovariance>>(*global_cloud, 0.7);
            pcl::PointCloud<pcl::PointCovariance>::Ptr source = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<PointXYZIRingTime>, pcl::PointCloud<pcl::PointCovariance>>(*cloud, 0.7);
    
            // Estimate covariances of points.
            const int num_threads = 6;
            const int num_neighbors = 10;
            small_gicp::estimate_covariances_omp(*target, num_neighbors, num_threads);
            small_gicp::estimate_covariances_omp(*source, num_neighbors, num_threads);
    
            // Create KdTree for target and source.
            auto target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(target, small_gicp::KdTreeBuilderOMP(num_threads));
            auto source_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source, small_gicp::KdTreeBuilderOMP(num_threads));
    
            small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
            registration.reduction.num_threads = num_threads;
            registration.rejector.max_dist_sq = 3.0;
    
            // Align point clouds. Note that the input point clouds are pcl::PointCloud<pcl::PointCovariance>.
            auto initial_guess = computeRigidTransformation(x, y, yaw, kDistanceLidarToCoG);
            auto result = registration.align(*target, *source, *target_tree, initial_guess);
            
            auto estimated_transformation = result.T_target_source;
            Eigen::Matrix4f transform_matrix = estimated_transformation.matrix().cast<float>();
    
            // Create an accumulated cloud
            pcl::PointCloud<PointXYZIRingTime>::Ptr accumulated_cloud(new pcl::PointCloud<PointXYZIRingTime>());
    
            // Add the latest frame (unmodified)
            pcl::transformPointCloud(*cloud, *cloud, transform_matrix);
            *global_cloud += *cloud;

        //Add the latest frame
        cloud_buffer.push_back(cloud);

        // Revert to original position before returning
        pcl::PointCloud<PointXYZIRingTime>::Ptr local_cloud(new pcl::PointCloud<PointXYZIRingTime>);
        pcl::transformPointCloud(*global_cloud, *local_cloud, final_transform.inverse().matrix());
        
        return local_cloud;
    }



    pcl::PointCloud<PointXYZIRingTime>::Ptr accumulate_cloud_small_icp(pcl::PointCloud<PointXYZIRingTime>::Ptr cloud, int kBufferSize,
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

        if (cloud_buffer.size() < 2) return cloud;

        // IMPLEMENTATION SMALL_GICP
        pcl::PointCloud<pcl::PointCovariance>::Ptr target = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<PointXYZIRingTime>, pcl::PointCloud<pcl::PointCovariance>>(*cloud, 0.3);
        pcl::PointCloud<pcl::PointCovariance>::Ptr source = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<PointXYZIRingTime>, pcl::PointCloud<pcl::PointCovariance>>(*cloud_buffer[cloud_buffer.size() - 1], 0.3);

        // Estimate covariances of points.
        const int num_threads = 6;
        const int num_neighbors = 10;
        small_gicp::estimate_covariances_omp(*target, num_neighbors, num_threads);
        small_gicp::estimate_covariances_omp(*source, num_neighbors, num_threads);

        // Create KdTree for target and source.
        auto target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(target, small_gicp::KdTreeBuilderOMP(num_threads));
        auto source_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source, small_gicp::KdTreeBuilderOMP(num_threads));

        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
        registration.reduction.num_threads = num_threads;
        registration.rejector.max_dist_sq = 3.0;

        // Align point clouds. Note that the input point clouds are pcl::PointCloud<pcl::PointCovariance>.
        auto initial_guess = computeRigidTransformation(vx, vy, yaw_rate, dt);
        auto result = registration.align(*target, *source, *target_tree, initial_guess);
        
        auto estimated_transformation = result.T_target_source;
        Eigen::Matrix4f transform_matrix = estimated_transformation.matrix().cast<float>();

        // Create an accumulated cloud
        pcl::PointCloud<PointXYZIRingTime>::Ptr accumulated_cloud(new pcl::PointCloud<PointXYZIRingTime>());

        //Iterate through stored frames (excluding the latest)
        for (size_t i = 0; i < cloud_buffer.size() - 2; ++i) 
        {
            if (!cloud_buffer[i] || cloud_buffer[i]->empty()) 
            {
                continue;
            }
            //Apply the transformation
            pcl::transformPointCloud(*cloud_buffer[i], *cloud_buffer[i], transform_matrix);

            //Merge into final accumulated cloud
            *accumulated_cloud += *cloud_buffer[i];
        }
        // Add the latest frame (unmodified)
        pcl::transformPointCloud(*cloud_buffer[cloud_buffer.size() - 2], *cloud_buffer[cloud_buffer.size() - 2], transform_matrix);
        *accumulated_cloud += *cloud_buffer[cloud_buffer.size() - 2];
        *accumulated_cloud += *cloud_buffer.back();

        return accumulated_cloud;
    }
}