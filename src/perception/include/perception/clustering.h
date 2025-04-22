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

    void parallel_euclidean_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices)
    {
        int number_sections = 2;
        double Mx = 30.0;
        double My = 15.0;
        double Mz = 0.5;
        
        double x_step = Mx / number_sections;
        double y_step = 2 * My / number_sections;
        
        // #pragma omp parallel for
        for (int i = 0; i < number_sections; ++i)
        {
            for (int j = 0; j < number_sections; ++j)
            {
                std::cout << "Sección " << i << " " << j << std::endl;

                // int i = index / number_sections;
                // int j = index % number_sections;

                Eigen::Vector4f min_pt(i * x_step, -My + j * y_step, -100.0, 1.0);
                Eigen::Vector4f max_pt((i + 1) * x_step, -My + (j + 1) * y_step, Mz, 1.0);

                pcl::PointCloud<pcl::PointXYZI>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::CropBox<pcl::PointXYZI> crop_box_filter;
                crop_box_filter.setInputCloud(cloud);
                crop_box_filter.setMin(min_pt);
                crop_box_filter.setMax(max_pt);
                crop_box_filter.filter(*subcloud);

                if (subcloud->empty()) continue;

                pcl::search::Octree<pcl::PointXYZI>::Ptr tree(new pcl::search::Octree<pcl::PointXYZI>(0.3));
                tree->setInputCloud(subcloud);

                pcl::EuclideanClusterExtraction<pcl::PointXYZI> extraction;
                extraction.setClusterTolerance(0.6);
                extraction.setMinClusterSize(4);
                extraction.setMaxClusterSize(200);
                extraction.setSearchMethod(tree);
                extraction.setInputCloud(subcloud);

                std::vector<pcl::PointIndices> local_clusters;
                extraction.extract(local_clusters);

                std::cout << "Number of clusters in section " << i << " " << j << ": " << local_clusters.size() << std::endl;

                // #pragma omp critical
                // {
                    cluster_indices.insert(cluster_indices.end(), local_clusters.begin(), local_clusters.end());
                // }

                std::cout << "Number of total clusters: " << cluster_indices.size() << std::endl;
                std::cout << "----------------------------------------" << std::endl;
            }
        }
    }
}