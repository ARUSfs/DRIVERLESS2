/**
 * @file color_estimation.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Contains auxiliar functions used in the algorithm to estimate the color of the closest cones.
 */

 #include "PointXYZColorScore.h"


 namespace ColorEstimation
 {
    /**
    * @brief Estimates the color of detected cones based on their intensity values. It filters cones within a given distance, 
    * segments them into vertical layers, and calculates the average intensity of the middle layers.
    */
    void color_estimation(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_points, std::vector<PointXYZColorScore>& clusters_centers,
        double distance_threshold, double coloring_threshold)
    {
        // Define the variables
        std::vector<std::pair<double, size_t>> averages_id;
        std::vector<std::pair<double, double>> z_intensity;
        std::vector<double> temporal_intensities, layer_intensities, layers, middle_intensities;

        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            // Calculate the distance from the cone to the origin
            PointXYZColorScore& center = clusters_centers[i];
            const auto& cluster = cluster_points[i];
            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < distance_threshold)
            {
                for (const auto& point : cluster->points) 
                {
                    // Store the (z, intensity) pair of the point
                    z_intensity.emplace_back(point.z, point.intensity);
                }

                // Order the (z, intensity) pairs regarding the z values using a lambda function
                std::sort(z_intensity.begin(), z_intensity.end(), [](const std::pair<double, double>& a, const std::pair<double, double>& b) 
                {
                    return a.first < b.first;
                });

                for (size_t j = 0; j < z_intensity.size() - 1; ++j)
                {
                    // Store the intensity and calculate the differnce between the z coordinates
                    temporal_intensities.push_back(z_intensity[j].second);
                    double difference = z_intensity[j + 1].first - z_intensity[j].first;

                    // If the difference is large enough it means it is a new layer
                    if (difference > 0.01)
                    {
                        // Calculate the average layer intensity and store in the temporal_intensities vector
                        double layer_intensity = std::accumulate(temporal_intensities.begin(), temporal_intensities.end(), 0.0) / temporal_intensities.size();
                        layer_intensities.push_back(layer_intensity);

                        // Store the last z coordinate as layer representative
                        layers.push_back(z_intensity[j + 1].first);

                        temporal_intensities.clear();
                    }
                }
                
                if (layer_intensities.empty()) continue;

                // Divide the layer in three equal parts
                size_t n = std::round(layers.size() / 3);
                for (size_t k = n; k < 2 * n; ++k)
                {
                    middle_intensities.push_back(layer_intensities[k]);
                }

                if (middle_intensities.empty()) continue;

                // Calculate the average layer intensity of the center layers of the cone and store it the averages_id vector
                double average_intensity = std::accumulate(middle_intensities.begin(), middle_intensities.end(), 0.0) / middle_intensities.size();
                averages_id.emplace_back(average_intensity, i);
            }

            z_intensity.clear();
            temporal_intensities.clear();
            layer_intensities.clear();
            layers.clear();
            middle_intensities.clear();
        }

        if (averages_id.empty()) return;

        // Calculate the average layer intensity of every cone processed before using a lambda function
        double global_sum = std::accumulate(averages_id.begin(), averages_id.end(), 0.0, [](double acc, const std::pair<double, size_t>& p) 
        {
            return acc + p.first;
        });
        double global_average_intensity = global_sum / averages_id.size();

        for (const auto& pair : averages_id)
        {
            size_t id = pair.second;

            // If the cone has a much larger intensity than the global average it means is a left cone
            if (pair.first > global_average_intensity + coloring_threshold)
            {
                clusters_centers[id].color = 1;
            }
            // If the cone has a much larger intensity than the global average it means is a right cone
            else if (pair.first < global_average_intensity - coloring_threshold)
            {
                clusters_centers[id].color = 2;
            }
        }
    }
 }