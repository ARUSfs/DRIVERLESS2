/**
 * @file scoring.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 */
 

namespace Utils 
{
    /**
    * @brief Extract the center of each cluster.
    */
    void get_clusters_centers(std::vector<pcl::PointIndices>& cluster_indices,
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& clusters_centers)
    {
        for (auto it = cluster_indices.begin(); it != cluster_indices.end(); )
        {
            // Create a temporal point cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*cloud_filtered, *it, *cluster_cloud);

            // Obtain the new bounding box of the cluster
            pcl::PointXYZI min_point, max_point;
            pcl::getMinMax3D(*cluster_cloud, min_point, max_point);
            double max_x = max_point.x;
            double min_x = min_point.x;
            double max_y = max_point.y;
            double min_y = min_point.y;
            double max_z = max_point.z;
            double min_z = min_point.z;

            // Filter the cluster by size and keep the center of the cluster
            if ((max_z - min_z) < 0.5 && min_z < 1.0 && (max_x - min_x)<  0.6 && (max_y - min_y) < 0.6)
            {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cluster_cloud, centroid);
                PointXYZColorScore center;
                center.x = centroid[0];
                center.y = centroid[1];
                center.z = min_z;
                center.color = 0;
                center.score = 0;
                clusters_centers.push_back(center);

                it++;
            }
            else
            {
                it = cluster_indices.erase(it);
            }
        }

        // Resize the cluster indices vector
        cluster_indices.resize(clusters_centers.size());
    }


    /**
    * @brief Recover falsely ground filtered points.
    */
    void reconstruction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, 
        std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore> clusters_centers, 
        double radius)
    {
        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            // Convert from PointXYZColorScore to PointXYZI
            pcl::PointXYZI center;
            center.x = clusters_centers[i].x;
            center.y = clusters_centers[i].y;
            center.z = clusters_centers[i].z;
            center.intensity = clusters_centers[i].score;
            
            for (size_t j = 0; j < cloud_plane->size(); ++j)
            {
                pcl::PointXYZI point = cloud_plane->points[j];

                // Check if the point lies inside the cylinder
                double dx = point.x - center.x;
                double dy = point.y - center.y;
                double radial_distance = std::sqrt(dx * dx + dy * dy);

                if (radial_distance <= radius)
                {
                    // Add the point to the filtered cloud
                    cloud_filtered->points.push_back(point);
                    cloud_filtered->width++;

                    // Add the index of the recovered point to cluster indices
                    cluster_indices[i].indices.push_back(cloud_filtered->points.size() - 1);
                }
            }
        }
    }


    /**
    * @brief Filter the final clusters by size to delete the ones that are too small or too large to be considered cones.
    */
    void filter_clusters(std::vector<pcl::PointIndices>& cluster_indices,
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& clusters_centers)
    {
        for (int i = cluster_indices.size() - 1; i >= 0; i--)
        {
            // Create a temporal point cloud
            pcl::PointIndices indices = cluster_indices[i];
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*cloud_filtered, indices, *cluster_cloud);

            // Obtain the new bounding box of the cluster
            pcl::PointXYZI min_point, max_point;
            pcl::getMinMax3D(*cluster_cloud, min_point, max_point);
            double max_x = max_point.x;
            double min_x = min_point.x;
            double max_y = max_point.y;
            double min_y = min_point.y;
            double max_z = max_point.z;
            double min_z = min_point.z;

            // Filter the cluster by size and delete the ones that are too small or too large
            if ((max_z - min_z) < 0.10 || (max_z - min_z) > 0.5 || (max_x - min_x) > 0.6 || (max_y - min_y) > 0.6)
            {
                clusters_centers.erase(clusters_centers.begin() + i);
                cluster_indices.erase(cluster_indices.begin() + i);
            }
        }
        
        // Resize the vectors
        clusters_centers.resize(clusters_centers.size());
        cluster_indices.resize(clusters_centers.size());
    }


    /**
    * @brief Correct the motion of the points in the final map.
    */
    void motion_correction(pcl::PointCloud<PointXYZColorScore>::Ptr cloud, double vx, double vy, double yaw_rate, double dt)
    {
        // Estimate the motion of the points
        double theta = -yaw_rate * dt; // Estimate DSK delay

        // Apply the estimated motion to the points
        for (auto& p : cloud->points)
        {
            double dx = vx * dt, dy = vy * dt;
            p.x = p.x * std::cos(theta) - p.y * std::sin(theta) - dx;
            p.y = p.x * std::sin(theta) + p.y * std::cos(theta) - dy;
        }
    } 
}