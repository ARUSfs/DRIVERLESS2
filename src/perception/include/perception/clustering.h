/**
 * @file clustering.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Contains auxiliar functions used in the algorithm to make the clusters from the filtered point cloud.
 */

#include <pcl/search/octree.h>
#include <pcl/segmentation/extract_clusters.h>
#include "fast_euclidean_clustering.h"

namespace Clustering
{
    /**
    * @brief Implements optimized Euclidean clustering using Octree search and OpenMP.
    */
    template<typename PointT>
    void EC(typename pcl::PointCloud<PointT>::Ptr& cloud, std::vector<typename pcl::PointCloud<PointT>::Ptr>* cluster_clouds,
        double tolerance, int min_cluster_size, int max_cluster_size)
    {
        if (!cloud || cloud->empty()) return;

        
        // Define the tree
        typename pcl::search::Octree<PointT>::Ptr tree(new pcl::search::Octree<PointT>(0.3));
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;

        // Configure the extraction
        pcl::EuclideanClusterExtraction<PointT> extraction;
        extraction.setClusterTolerance(tolerance);  
        extraction.setMinClusterSize(min_cluster_size);     
        extraction.setMaxClusterSize(max_cluster_size);
        extraction.setSearchMethod(tree);

        // Apply the extraction
        extraction.setInputCloud(cloud);
        extraction.extract(cluster_indices);

        for (auto it = cluster_indices.begin(); it != cluster_indices.end(); )
        {
            typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
            pcl::copyPointCloud(*cloud, *it, *cluster_cloud);
            cluster_clouds->push_back(cluster_cloud);
            it++;
        }
    }


    template<typename PointT>
    void FEC(typename pcl::PointCloud<PointT>::Ptr& cloud, std::vector<typename pcl::PointCloud<PointT>::Ptr>* cluster_clouds,
        double tolerance, int min_cluster_size, int max_cluster_size)
    {
        if (!cloud || cloud->empty()) return;

        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        std::vector<pcl::PointIndices> cluster_indices;

        FastEuclideanClustering<PointT> fec;
        fec.setInputCloud(cloud);
        fec.setSearchMethod(tree);
        fec.setClusterTolerance(tolerance);
        fec.setQuality(0.5);
        fec.setMinClusterSize(min_cluster_size);
        fec.setMaxClusterSize(max_cluster_size);
        fec.segment(cluster_indices);

        for (auto it = cluster_indices.begin(); it != cluster_indices.end(); )
        {
            typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
            pcl::copyPointCloud(*cloud, *it, *cluster_cloud);
            cluster_clouds->push_back(cluster_cloud);
            it++;
        }
    }
}
