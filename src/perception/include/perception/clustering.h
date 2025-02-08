/**
 * @file clustering.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to make the clusters from the filtered point cloud.
 * @version 0.1
 * @date 15-11-2024
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Clustering
{
    /**
    * @brief Implements euclidean clustering.
    * @param cloud The filtered point cloud from which we will extract the clusters.
    * @param cluster_indices The vector that will store the different indices of the clusters points.
    */
    void euclidean_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices)
    {
        //Create the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud);

        //configure the algorithm
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> extraction;
        extraction.setClusterTolerance(0.6);
        extraction.setMinClusterSize(4);
        extraction.setMaxClusterSize(200);
        extraction.setSearchMethod(tree);

        //Apply the extraction
        extraction.setInputCloud(cloud);
        extraction.extract(cluster_indices);
    }
}