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
    void euclidean_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices)
    {
        if (!cloud || cloud->empty()) return;

        pcl::search::Octree<pcl::PointXYZI>::Ptr tree(new pcl::search::Octree<pcl::PointXYZI>(0.3));
        tree->setInputCloud(cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> extraction;
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
}