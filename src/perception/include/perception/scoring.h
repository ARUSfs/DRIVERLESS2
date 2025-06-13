/**
 * @file scoring.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Contains auxiliar functions used in the algorithm to score every cluster in order to determine the real cones.
 */


#include "PointXYZProbColorScore.h"
#include "PointXYZIRingTime.h"


namespace Scoring
{
    /**
    * @brief Score every cluster by how similar its to a cone and based on the distance between
    * its points and what would be the real cone, and then only keep the ones that a have a certain similarity to be considered cones.
    */
    void scoring_surface(pcl::PointCloud<PointXYZProbColorScore>::Ptr& final_map, std::vector<pcl::PointCloud<PointXYZIRingTime>::Ptr> cluster_points, 
        std::vector<PointXYZIRingTime>& cluster_centers, double threshold)
    {
        // Define the cone
        const double kBaseRadius = 0.07525;  
        const double kHeight = 0.40;      

        for (size_t i = 0; i < cluster_points.size(); ++i) 
        {
            // Define the variables
            PointXYZIRingTime& center = cluster_centers[i];
            const auto& cluster = cluster_points[i];
            double total_score = 0.0;
            double cone_radius_at_z;

            for (const auto& point : cluster->points)    
            {
                // Calculate the distante in x,y to the center of the cone
                double dx = point.x - center.x;
                double dy = point.y - center.y;

                // Calculate the distance in z to the base of the cone
                double dz = point.z - center.z;

                // Determinate the cone radius depending on the height in z
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

                // Calulate the distante to the surface of the cone
                double horizontal_distance = std::sqrt(dx * dx + dy * dy);
                double distance_to_cone_surface = std::abs(horizontal_distance - cone_radius_at_z) * std::cos(0.1868);
                
                // Apply the formula to get the score
                double score = 1.0 - std::min(std::pow(distance_to_cone_surface,2) / std::pow(0.1,2), 1.0);

                total_score += score;
            }

            // Find the score of the cluster
            double average_score = total_score / cluster->size();
            
            // Filter by the threshold and keep the clusters that will be cones
            if (average_score >= threshold)
            {   
                PointXYZProbColorScore cone = PointXYZProbColorScore(center.x, center.y, center.z, 0.0, 0.0, average_score);
                final_map->points.push_back(cone);
            }
        }
    }
}