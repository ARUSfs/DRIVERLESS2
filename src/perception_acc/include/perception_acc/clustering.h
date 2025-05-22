/**
 * @file clustering.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to make the clusters from the filtered point cloud.
 * @version 0.2
 * @date 09-03-2024
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/octree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <omp.h>

namespace Clustering
{
    /**
    * @brief Implements optimized Euclidean clustering using Octree search and OpenMP.
    * @param cloud The filtered point cloud from which we will extract the clusters.
    * @param cluster_indices The vector that will store the different indices of the clusters points.
    */
    void euclidean_clustering(pcl::PointCloud<PointXYZIRingTime>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices)
    {
        if (!cloud || cloud->empty()) return;

        pcl::search::Octree<PointXYZIRingTime>::Ptr tree(new pcl::search::Octree<PointXYZIRingTime>(0.3));
        tree->setInputCloud(cloud);

        pcl::EuclideanClusterExtraction<PointXYZIRingTime> extraction;
        extraction.setClusterTolerance(0.6);  
        extraction.setMinClusterSize(4);     
        extraction.setMaxClusterSize(200);
        extraction.setSearchMethod(tree);

        extraction.setInputCloud(cloud);
        #pragma omp parallel
        {
            #pragma omp single
            extraction.extract(cluster_indices);
        }
    }

    /**
     * @brief Extract the center of each cluster.
     * @param cluster_indices The indices of the points that form each cluster.
     * @param cloud_filtered The input point cloud.
     * @param cluster_centers The center of each cluster.
     */
    void get_clusters_centers(std::vector<pcl::PointIndices>& cluster_indices,
        pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& clusters_centers)
    {
        for (auto it = cluster_indices.begin(); it != cluster_indices.end(); )
        {
            //Create a temporal point cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*cloud_filtered, *it, *cluster_cloud);

            //Obtain the new bounding box of the cluster
            pcl::PointXYZI min_point, max_point;
            pcl::getMinMax3D(*cluster_cloud, min_point, max_point);
            double max_x = max_point.x;
            double min_x = min_point.x;
            double max_y = max_point.y;
            double min_y = min_point.y;
            double max_z = max_point.z;
            double min_z = min_point.z;

            //Filter the cluster by size and keep the center of the cluster
            if ((max_z-min_z)<0.4 && min_z<0.0 && (max_x-min_x)<0.5 && (max_y-min_y)<0.5)
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
        //Resize the cluster indices vector
        cluster_indices.resize(clusters_centers.size());
    }
}