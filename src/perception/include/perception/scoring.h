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

namespace Scoring
{
    /**
    * @brief Score every cluster by how similar its to a cone and based on the distance between
    * its points and what would be the real cone, and then only keep the ones that a have a certain similarity to be considered cones.
    * @param cloud_filtered The filtered point cloud.
    * @param final_map The point cloud where the centers of the clusters will be keep.
    * @param cluster_indices The indices of the points that form each cluster.
    * @param cluster_centers The center of each cluster.
    * @param threshold The threshold that wll the determinate if the cluster is a cone or not.
    */
    void scoring_surface(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered, 
        pcl::PointCloud<PointXYZColorScore>::Ptr& final_map, std::vector<pcl::PointIndices>& cluster_indices, 
        std::vector<PointXYZColorScore>& cluster_centers, double threshold)
    {
        //Define the cone
        const double kBaseRadius = 11.4;  
        const double kHeight = 32.5;      
        const double kTSquare = 1.0;   

        //iterate on each cluster
        for (size_t i = 0; i < cluster_indices.size(); ++i) 
        {
            PointXYZColorScore& center = cluster_centers[i];
            const auto& indices = cluster_indices[i];
            double total_score = 0.0;
            double cone_radius_at_z;

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);
            
            //std::cout << "Distance: " << distance << std::endl;

            //Calculate the height of the base
            double min_z = std::numeric_limits<double>::max();
            for (const auto& index : indices.indices) 
            {
                const auto& point = cloud_filtered->points[index];
                if (point.z < min_z) 
                {
                    min_z = point.z;
                }
            }

            //Iterate on each point
            for (const auto& index : indices.indices)        
            {
                const auto& point = cloud_filtered->points[index];

                //Calculate the distante in x,y to the center of the cone
                double dx = point.x - center.x;
                double dy = point.y - center.y;

                //Calculate the distance in z to the base of the cone
                double dz = point.z - min_z;

                //Determinate the cone radius depending on the height in z
                if (dz > kHeight)
                {
                    cone_radius_at_z = 0.0;
                }
                else if (dz <= 0.0)
                {
                    cone_radius_at_z = kBaseRadius;
                }   
                else
                {
                    cone_radius_at_z = (dz / kHeight) * kBaseRadius;
                }

                //Calulate the distante to the surface of the cone
                double horizontal_distance = std::sqrt(dx * dx + dy * dy);
                double distance_to_cone_surface = std::abs(horizontal_distance - cone_radius_at_z);

                //Apply the formula to get the score
                double score = 1.0 - std::min(distance_to_cone_surface * distance_to_cone_surface / kTSquare, 1.0);
                total_score += score;
            }

            //Find the score of the cluster
            double average_score = total_score / indices.indices.size();

            //std::cout << "Score: " << average_score << std::endl;

            const double min_threshold = 0.85;
            const double max_threshold = 0.95;
            
            double dynamic_threshold = std::max(min_threshold, max_threshold - (distance / 100));

            //std::cout << "Threshold: " << dynamic_threshold << std::endl;

            //std::cout << "********************************************" << std::endl;

            //Filter by the threshold and keep the clusters that will be cones
            if (average_score >= dynamic_threshold) 
            {
                center.score = average_score;
                final_map->points.push_back(center);
            }
        }
    }
}