/**
 * @file scoring.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to score every cluster in order to determine the real cones.
 * @version 0.1
 * @date 3-12-2024
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "PointXYZColorScore.h"

namespace Scoring{
    /**
    * @brief Calculate the deviation of a vector.
    * @param values The vector containing the numbers.
    */
    double deviation(std::vector<double> values)
    {
        //Calculate the mean of the values
        double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();

        //Calculate the square of the sum
        double sq_sum = 0.0;
        for (const auto& value : values)
        {
            sq_sum += std::pow(value - mean, 2);
        }

        //Returns the deviation
        return std::sqrt(sq_sum / values.size());
    }

    /**
    * @brief Score every cluster by how similar its to a cone and based on the deviation
    * of its points and only keep the ones that a have a certain similarity to be considered cones.
    * @param cloud_filtered The filtered point cloud.
    * @param cloud_clusters The point cloud where the clusters will be keep.
    * @param filtered_map_cloud The point cloud where the centers of the clusters will be keep.
    * @param cluster_indices The indices of the points that form each cluster.
    * @param cluster_centers The center of each cluster.
    */
    void scoring_deviation(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_clusters, 
        pcl::PointCloud<PointXYZColorScore>::Ptr& filtered_map_cloud, std::vector<pcl::PointIndices>& cluster_indices, 
        std::vector<PointXYZColorScore>& cluster_centers, double minimum_score, double maximum_score)
    {
        //Interate on clusters
        for (int i = cluster_indices.size() - 1; i >= 0; --i)  
        {
            const auto& indices = cluster_indices[i].indices;

            std::vector<double> dispersion_xy;
            std::vector<double> dispersion_z;

            //Calculates the dispersion on xy and z axis
            for (const auto& j : indices)
            {
                const auto& point = cloud_filtered->points[j];

                double dispersion_xy_point = std::sqrt(point.x*point.x + point.y * point.y);
                dispersion_xy.push_back(dispersion_xy_point);

                dispersion_z.push_back(point.z);
            }

            //Calculates the deviation on xy and z axis by calling the deviation function
            double deviation_xy = Scoring::deviation(dispersion_xy);
            double deviation_z = Scoring::deviation(dispersion_z);
        
            //Calculates the score of the cluster
            double taper_index = deviation_xy / deviation_z;

            //Filter the cluster by their score and keep the ones with a certain score
            if (taper_index > -1.0 && taper_index < 1.0)
            {
                for (const auto& index : indices)
                {
                    cloud_clusters->points.push_back(cloud_filtered->points[index]);
                }

                cluster_centers[i].score = taper_index;
                filtered_map_cloud->push_back(cluster_centers[i]);
            }
        }
    }

}