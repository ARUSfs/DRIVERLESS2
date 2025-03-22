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

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

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

    /**
    * @brief Accumulate the clouds of the last frames.
    * @param cloud The clouds you want to store in the buffer.
    * @param kBufferSize The size of cloud buffers.
    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulate_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int kBufferSize,
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

        rigidTransformation(cloud_buffer[cloud_buffer.size() - 2], vx, vy, yaw_rate, dt);

        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setMaximumIterations(50);
        icp.setInputSource(cloud_buffer[cloud_buffer.size() - 2]);
        icp.setInputTarget(cloud);
        icp.setEuclideanFitnessEpsilon(0.005);
        icp.setTransformationEpsilon(1e-5);
        icp.setMaxCorrespondenceDistance(0.05);

        Eigen::Matrix4f icp_transform = icp.getFinalTransformation();

        // Create an accumulated cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        //Iterate through stored frames (excluding the latest)
        for (size_t i = 0; i < cloud_buffer.size() - 2; ++i) 
        {
            if (!cloud_buffer[i] || cloud_buffer[i]->empty()) 
            {
                continue;
            }
            //Apply the transformation
            rigidTransformation(cloud_buffer[i], vx, vy, yaw_rate, dt);
            pcl::transformPointCloud(*cloud_buffer[i], *cloud_buffer[i], icp_transform);

            //Merge into final accumulated cloud
            *accumulated_cloud += *cloud_buffer[i];
        }
        // Add the latest frame (unmodified)
        pcl::transformPointCloud(*cloud_buffer[cloud_buffer.size() - 2], *cloud_buffer[cloud_buffer.size() - 2], icp_transform);
        *accumulated_cloud += *cloud_buffer[cloud_buffer.size() - 2];
        *accumulated_cloud += *cloud_buffer.back();

        return accumulated_cloud;
    }





    /**
     * @brief Transforms a PointCloud using a 6x6 Hessian matrix (rotation and translation).
     * @param cloud_in Input point cloud (pcl::PointCloud<pcl::PointXYZI>)
     * @param H Hessian matrix (6x6), where [rx, ry, rz] are rotation and [tx, ty, tz] are translation
     * @return Transformed point cloud
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloudWithHessian(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
        const Eigen::Matrix<double, 6, 6>& H) 
    {

        // Extract rotation (rx, ry, rz) and translation (tx, ty, tz)
        Eigen::Vector3d rotation_vec(H(0, 5), H(1, 5), H(2, 5));  // Rotation vector
        Eigen::Vector3d translation(H(3, 5), H(4, 5), H(5, 5));   // Translation vector

        // Convert rotation vector to rotation matrix using SO(3) exponential map
        Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(rotation_vec.norm(), rotation_vec.normalized()).toRotationMatrix();

        // Construct 4x4 transformation matrix
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        transformation.block<3, 3>(0, 0) = rotation_matrix.cast<float>();  // Rotation
        transformation.block<3, 1>(0, 3) = translation.cast<float>();      // Translation

        // Apply transformation to the point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*cloud_in, *cloud_out, transformation);

        return cloud_out;
    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulate_cloud_small_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int kBufferSize,
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

        rigidTransformation(cloud_buffer[cloud_buffer.size() - 2], vx, vy, yaw_rate, dt);

        // small_gicp::voxelgrid_downsampling can directly operate on pcl::PointCloud.
        pcl::PointCloud<pcl::PointXYZI>::Ptr target = small_gicp::voxelgrid_sampling_omp(*cloud, 0.25);
        pcl::PointCloud<pcl::PointXYZI>::Ptr source = small_gicp::voxelgrid_sampling_omp(*cloud_buffer[cloud_buffer.size() - 2], 0.25);

        // RegistrationPCL is derived from pcl::Registration and has mostly the same interface as pcl::GeneralizedIterativeClosestPoint.
        small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI> reg;
        reg.setNumThreads(16);
        reg.setCorrespondenceRandomness(20);
        reg.setMaxCorrespondenceDistance(1.0);
        reg.setVoxelResolution(1.0);
        reg.setRegistrationType("GGICP");  // or "GICP" (default = "GICP")

        // Set input point clouds.
        reg.setInputTarget(target);
        reg.setInputSource(source);
        
        auto H = reg.getFinalHessian();

        // Create an accumulated cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        //Iterate through stored frames (excluding the latest)
        for (size_t i = 0; i < cloud_buffer.size() - 2; ++i) 
        {
            if (!cloud_buffer[i] || cloud_buffer[i]->empty()) 
            {
                continue;
            }
            //Apply the transformation
            rigidTransformation(cloud_buffer[i], vx, vy, yaw_rate, dt);
            cloud_buffer[i] = transformPointCloudWithHessian(cloud_buffer[i], H);

            //Merge into final accumulated cloud
            *accumulated_cloud += *cloud_buffer[i];
        }
        // Add the latest frame (unmodified)
        cloud_buffer[cloud_buffer.size() - 2] = transformPointCloudWithHessian(cloud_buffer[cloud_buffer.size() - 2], H);
        *accumulated_cloud += *cloud_buffer[cloud_buffer.size() - 2];
        *accumulated_cloud += *cloud_buffer.back();

        return accumulated_cloud;
    }



    Eigen::Isometry3d computeRigidTransformation(double vx, double vy, double yaw_rate, double dt) {
        // Compute translation and rotation
        double delta_x = -vx * dt;
        double delta_y = -vy * dt;
        double delta_theta = -yaw_rate * dt;

        // Transformation from LIDAR to CoG
        Eigen::Isometry3d transform_lidar_to_CoG = Eigen::Isometry3d::Identity();
        transform_lidar_to_CoG.pretranslate(Eigen::Vector3d(-1.5, 0.0, 0.0));

        // Motion transformation matrix (rotation + translation)
        Eigen::Isometry3d transform_motion = Eigen::Isometry3d::Identity();
        transform_motion.linear() << cos(delta_theta), -sin(delta_theta), 0,
                                    sin(delta_theta),  cos(delta_theta), 0,
                                    0,                0,                1;
        transform_motion.pretranslate(Eigen::Vector3d(delta_x, delta_y, 0.0));

        // Transformation from CoG back to LIDAR
        Eigen::Isometry3d transform_CoG_to_lidar = Eigen::Isometry3d::Identity();
        transform_CoG_to_lidar.pretranslate(Eigen::Vector3d(1.5, 0.0, 0.0));

        // Compute final transformation matrix
        Eigen::Isometry3d final_transform = transform_lidar_to_CoG * transform_motion * transform_CoG_to_lidar;

        return final_transform;  // Return the transformation matrix
    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulate_cloud_small_icp2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int kBufferSize,
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
        pcl::PointCloud<pcl::PointCovariance>::Ptr target = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(*cloud, 0.25);
        pcl::PointCloud<pcl::PointCovariance>::Ptr source = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(*cloud_buffer[cloud_buffer.size() - 2], 0.25);

        // Estimate covariances of points.
        const int num_threads = 6;
        const int num_neighbors = 20;
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
        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>());

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

    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>accumulate_clusters(
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cluster_points, std::vector<PointXYZColorScore>&
            clusters_centers, int kBufferSize, double AccumulationThreshold,double vx, double vy, double yaw_rate, double dt)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> final_clusters;
        std::vector<PointXYZColorScore> final_centers;

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
        return final_clusters;
    }



    std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, std::vector<PointXYZColorScore>>
    accumulate_clusters_small_gicp(
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cluster_points,
        std::vector<PointXYZColorScore>& clusters_centers,
        int kBufferSize,
        double AccumulationThreshold,
        double vx, double vy, double yaw_rate, double dt)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> final_clusters;
        std::vector<PointXYZColorScore> final_centers;

        if (!buffer_cluster_initialized) {
            cluster_buffer.clear();
            center_buffer.clear();
            buffer_cluster_initialized = true;
        }

        if (cluster_buffer.size() >= static_cast<size_t>(kBufferSize)) {
            cluster_buffer.pop_front();
            center_buffer.pop_front();
        }

        cluster_buffer.push_back(cluster_points);
        center_buffer.push_back(clusters_centers);

        if (cluster_buffer.size() < 2)
            return {cluster_points, clusters_centers};

        pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        for (const auto& cluster : cluster_points)
            if (cluster) *target_cloud += *cluster;
        for (const auto& cluster : cluster_buffer[cluster_buffer.size() - 2])
            if (cluster) *source_cloud += *cluster;

        auto target = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(*target_cloud, 0.25);
        auto source = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(*source_cloud, 0.25);

        const int num_threads = 6, num_neighbors = 20;
        small_gicp::estimate_covariances_omp(*target, num_neighbors, num_threads);
        small_gicp::estimate_covariances_omp(*source, num_neighbors, num_threads);

        auto target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(target, small_gicp::KdTreeBuilderOMP(num_threads));
        auto source_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source, small_gicp::KdTreeBuilderOMP(num_threads));

        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
        registration.reduction.num_threads = num_threads;
        registration.rejector.max_dist_sq = 3.0;

        auto initial_guess = computeRigidTransformation(vx, vy, yaw_rate, dt);
        auto result = registration.align(*target, *source, *target_tree, initial_guess);
        Eigen::Matrix4f transform_matrix = result.T_target_source.matrix().cast<float>();

        std::vector<std::pair<size_t, size_t>> index_map;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> all_clusters;
        std::vector<PointXYZColorScore> all_centers;

        for (size_t i = 0; i < center_buffer.size(); ++i) {
            for (size_t j = 0; j < center_buffer[i].size(); ++j) {
                if (!cluster_buffer[i][j]) continue;

                pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cluster(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::transformPointCloud(*cluster_buffer[i][j], *transformed_cluster, transform_matrix);
                all_clusters.push_back(transformed_cluster);

                Eigen::Vector4f pt(center_buffer[i][j].x, center_buffer[i][j].y, center_buffer[i][j].z, 1.0);
                Eigen::Vector4f pt_tf = transform_matrix * pt;
                PointXYZColorScore new_center = center_buffer[i][j];
                new_center.x = pt_tf[0]; new_center.y = pt_tf[1]; new_center.z = pt_tf[2];
                all_centers.push_back(new_center);

                index_map.emplace_back(i, j);
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& c : all_centers)
            center_cloud->push_back(pcl::PointXYZ(c.x, c.y, c.z));

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(center_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(AccumulationThreshold);
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(kBufferSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(center_cloud);
        ec.extract(cluster_indices);

        for (const auto& group : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cluster(new pcl::PointCloud<pcl::PointXYZI>);
            PointXYZColorScore merged_center;
            int count = 0;

            for (int idx : group.indices) {
                if (idx < 0 || static_cast<size_t>(idx) >= index_map.size()) continue;
                auto [frame_idx, cluster_idx] = index_map[idx];
                if (!cluster_buffer[frame_idx][cluster_idx]) continue;

                *merged_cluster += *cluster_buffer[frame_idx][cluster_idx];

                const auto& center = center_buffer[frame_idx][cluster_idx];
                merged_center.x += center.x;
                merged_center.y += center.y;
                merged_center.z += center.z;
                count++;
            }

            if (count > 0) {
                merged_center.x /= count;
                merged_center.y /= count;
                merged_center.z /= count;

                final_clusters.push_back(merged_cluster);
                final_centers.push_back(merged_center);
            }
        }

        return {final_clusters, final_centers};
    }
}