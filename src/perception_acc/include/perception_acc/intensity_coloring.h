/**
 * @file intensity_coloring.h
 * @author Rafael Guil Valero (rafaguilvalero@gmail.com)
 * @brief Contains auxiliar functions used in the algorithm to estimate the color of the closest cones.
 */

 #include "PointXYZColorScore.h"


 namespace IntensityColoring
 {
    /**
    * @brief Estimates the color of detected cones based on intensity. 
    */
    void color_estimation(std::vector<pcl::PointCloud<PointXYZIRingTime>::Ptr> cluster_points, std::vector<PointXYZColorScore>& clusters_centers,
        double distance_threshold, pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_plane)
    {
        // Find average intensity of the ground
        double ground_intensity = 0.0;
        for(const auto& pt : cloud_plane->points)
        {
            ground_intensity += pt.intensity;
        }
        ground_intensity /= cloud_plane->size();
        std::cout << "Ground intensity: " << ground_intensity << std::endl;

        
        for(size_t i = 0; i < cluster_points.size(); i++)
        {
            if(std::sqrt(std::pow(clusters_centers[i].x, 2) + std::pow(clusters_centers[i].y, 2)) > distance_threshold ||
                std::sqrt(std::pow(clusters_centers[i].x, 2) + std::pow(clusters_centers[i].y, 2)) < 3.0)
            {
                clusters_centers[i].color = 0; // unknown
                continue;
            }

            double average_intensity = 0.0;
            for(const auto& pt : cluster_points[i]->points)
            {
                average_intensity += pt.intensity;
            }
            average_intensity /= cluster_points[i]->points.size();

            double color_param = ground_intensity + 5;
            double coloring_scoring = average_intensity / color_param;
            clusters_centers[i].color = coloring_scoring;

            std::cout << "x: " << clusters_centers[i].x << "; y: " << clusters_centers[i].y << "; Coloring Scoring: " << coloring_scoring << "; cluster_points[i]->points.size(): " << cluster_points[i]->points.size() << "; Average intensity: " << average_intensity << "; ground_intensity: " << ground_intensity << std::endl;
        }
    }
 }