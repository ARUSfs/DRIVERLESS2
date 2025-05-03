/**
 * @file clustering.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Contains auxiliar functions used in the algorithm to make the clusters from the filtered point cloud.
 */


#include <pcl/search/octree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <omp.h>


namespace Clustering
{
    /**
    * @brief Implements optimized Euclidean clustering using Octree search and OpenMP.
    */
    void euclidean_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices)
    {
        if (!cloud || cloud->empty()) return;

        // Define the tree
        pcl::search::Octree<pcl::PointXYZI>::Ptr tree(new pcl::search::Octree<pcl::PointXYZI>(0.3));
        tree->setInputCloud(cloud);

        // Configure the extraction
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> extraction;
        extraction.setClusterTolerance(0.6);  
        extraction.setMinClusterSize(4);     
        extraction.setMaxClusterSize(200);
        extraction.setSearchMethod(tree);

        // Apply the extraction
        extraction.setInputCloud(cloud);
        #pragma omp parallel // OMP no hace nada aquí
        {
            #pragma omp single
            extraction.extract(cluster_indices);
        }
    }
}