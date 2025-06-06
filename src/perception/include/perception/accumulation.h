/**
 * @file accumulation.h
 * @author Álvaro Galisteo Bermúdez (galisbermo03@gmail.com)
 * @brief Contains auxiliar functions used in the algorithm to accumulate the point cloud received from the lidar.
 */


#include <Eigen/Dense>
#include <deque>


namespace Accumulation
{
    static std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_buffer;
    static bool buffer_cloud_initialized = false;

    static std::deque<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> cluster_buffer;
    static std::deque<std::vector<PointXYZProbColorScore>> center_buffer;
    static bool buffer_cluster_initialized = false;

    /**
    * @brief Function to apply rigid transformation to a point cloud
    */
    void rigidTransformation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
            double vx, double vy, double yaw_rate, double dt)
    {
        double delta_x = -vx * dt;
        double delta_y = -vy * dt;
        double delta_theta = -yaw_rate * dt;

        Eigen::Matrix4f transform_lidar_to_CoG = Eigen::Matrix4f::Identity();
        transform_lidar_to_CoG(0, 3) = -1.5; 

        Eigen::Matrix4f transform_motion = Eigen::Matrix4f::Identity();
        transform_motion(0, 0) = cos(delta_theta);
        transform_motion(0, 1) = -sin(delta_theta);
        transform_motion(1, 0) = sin(delta_theta);
        transform_motion(1, 1) = cos(delta_theta);
        transform_motion(0, 3) = delta_x;
        transform_motion(1, 3) = delta_y;

        Eigen::Matrix4f transform_CoG_to_lidar = Eigen::Matrix4f::Identity();
        transform_CoG_to_lidar(0, 3) = 1.5;

        Eigen::Matrix4f final_transform = transform_lidar_to_CoG * transform_motion * transform_CoG_to_lidar;

        pcl::transformPointCloud(*cloud, *cloud, final_transform);
    }

    /**
    * @brief Accumulate the clouds of the last frames.
    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulate_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int kBufferSize,
            double vx, double vy, double yaw_rate, double dt)
    {
        // Clean the buffer the first time
        if (!buffer_cloud_initialized) {
            cloud_buffer.clear();
            buffer_cloud_initialized = true;
        }

        // Ensure buffer size limit
        if (cloud_buffer.size() >= static_cast<size_t>(kBufferSize)) 
        {
            cloud_buffer.pop_front();
        }

        // Add the latest frame
        cloud_buffer.push_back(cloud);

        // Create an accumulated cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // Iterate through stored frames (excluding the latest)
        for (size_t i = 0; i < cloud_buffer.size() - 1; ++i) 
        {
            if (!cloud_buffer[i] || cloud_buffer[i]->empty()) 
            {
                continue;
            }
            // Apply the transformation
            rigidTransformation(cloud_buffer[i], vx, vy, yaw_rate, dt);

            // Merge into final accumulated cloud
            *accumulated_cloud += *cloud_buffer[i];
        }
        // Add the latest frame (unmodified)
        *accumulated_cloud += *cloud_buffer.back();

        return accumulated_cloud;
    }

    /**
    * @brief Accumulates the clusters of the last 5 frames of the lidar call back.
    */
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>accumulate_clusters(
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cluster_points, std::vector<PointXYZProbColorScore>&
            clusters_centers, int kBufferSize, double AccumulationThreshold,double vx, double vy, double yaw_rate, double dt)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> final_clusters;
        std::vector<PointXYZProbColorScore> final_centers;

        // Clean the buffer the first time
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
        std::vector<PointXYZProbColorScore> all_centers;

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
}