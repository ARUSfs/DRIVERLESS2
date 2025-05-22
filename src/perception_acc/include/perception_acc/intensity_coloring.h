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
        double distance_threshold, double intensity_threshold)
    {
        static double highest_z = -std::numeric_limits<double>::infinity();

        // Find closest cluster and update highest_z
        {
            double min_dist = std::numeric_limits<double>::infinity();
            size_t closest_idx = 0;
            for(size_t i = 0; i < cluster_points.size(); i++)
            {
                double dist = std::sqrt(std::pow(clusters_centers[i].x, 2) + std::pow(clusters_centers[i].y, 2));
                if(dist < min_dist)
                {
                    min_dist = dist;
                    closest_idx = i;
                }
            }
            double temp_z = -std::numeric_limits<double>::infinity();
            for(const auto& pt : cluster_points[closest_idx]->points)
            {
                if(pt.z > temp_z) temp_z = pt.z;
            }
            highest_z = temp_z;
        }

        for(size_t i = 0; i < cluster_points.size(); i++)
        {
            if(std::sqrt(std::pow(clusters_centers[i].x, 2) + std::pow(clusters_centers[i].y, 2)) > distance_threshold ||
                std::sqrt(std::pow(clusters_centers[i].x, 2) + std::pow(clusters_centers[i].y, 2)) < 3.0)
            {
                clusters_centers[i].color = 0; // unknown
                continue;
            }

            double max_line_z = highest_z - 0.1;
            double min_line_z = highest_z - 0.15;
            std::vector<int> intensity_values;
            for(const auto& pt : cluster_points[i]->points)
            {
                if(pt.z <= min_line_z || pt.z >= max_line_z)
                {
                    intensity_values.push_back(pt.intensity);
                };
            }

            double average_intensity = 0.0;
            for(const auto& intensity : intensity_values)
            {
                average_intensity += intensity;
            }
            average_intensity /= intensity_values.size();
            // std::cout << "x: " << clusters_centers[i].x << "; y: " << clusters_centers[i].y << "; Highest z: " << highest_z << "; Intensity_values.size(): " << intensity_values.size() << "; Average intensity: " << average_intensity << "; intensity_threshold: " << intensity_threshold << std::endl;

            if (average_intensity >= intensity_threshold - 2 && average_intensity <= intensity_threshold + 2) clusters_centers[i].color = 0; // unknown
            else if(average_intensity > intensity_threshold) clusters_centers[i].color = 1; // blue
            else clusters_centers[i].color = 2; // yellow
        }
    }
 }