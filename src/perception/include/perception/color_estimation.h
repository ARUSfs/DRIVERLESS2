/**
 * @file color_estimation.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to estimate the color of the closest cones.
 * @version 0.1
 * @date 4-2-2025
 */

 namespace ColorEstimation
 {
    /**
    * @brief Estimates the color of detected cones based on their intensity values. Processes LiDAR point cloud data to classify cones as left or right 
    * by analyzing their intensity distribution. It filters cones within a given distance, segments them into vertical layers, and calculates the 
    * average intensity of the middle layers. Each cone's intensity is then compared to the global average to determine its classification:
    * - Higher intensity → Left cone (color = 1)  
    * - Lower intensity → Right cone (color = 2) 
    * @param cluster_indices The vector that stores the different indices of the clusters points.
    * @param clusters_centers The vector that stores the different centers of the cones.
    * @param cloud_filtered The cloud that contains the clusters points.
    * @param distance_threshold The distance that decides if the cone is going to be processed or not.
    * @param coloring_threshold The threshold between the global average intensity and the average intensity of each cone.
    */
    void color_estimation(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered,
        double distance_threshold, double coloring_threshold)
    {
        // Define a vector to store (average, id) pairs for each cone, where the id will be the index
        std::vector<std::pair<double, size_t>> averages_id;

        // Define a vector to store (z, intensity) pairs for each point
        std::vector<std::pair<double, double>> z_intensity;

        // Define the vectors that will keep the average intensity and the height of every layers, 
        // and the average intensity in the middle layers 
        std::vector<double> temporal_intensities, layer_intensities, layers, middle_intensities;

        // Iterate on each cone
        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            // Define the center and the indices of the cone
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            // Calculate the distance from the cone to the origin
            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            // Process the cone only if it is within the specified distance threshold
            if (distance < distance_threshold)
            {
                std::cout << "Distancia: " << distance << std::endl;
   
                // Iterate on each point of the cluster
                for (const auto& index : indices.indices) 
                {
                    // Store the (z, intensity) pair of the point
                    const auto& point = cloud_filtered->points[index];
                    z_intensity.emplace_back(point.z, point.intensity);
                }

                // Order the (z, intensity) pairs regarding the z values using a lambda function
                std::sort(z_intensity.begin(), z_intensity.end(), [](const std::pair<double, double>& a, const std::pair<double, double>& b) 
                {
                    return a.first < b.first;
                });

                // Iterate on every (z, intensity) pair
                for (size_t j = 0; j < z_intensity.size() - 1; ++j)
                {
                    // Calculate the difference between the current z and the next one
                    double difference = z_intensity[j + 1].first - z_intensity[j].first;

                    // Store the intensity 
                    temporal_intensities.push_back(z_intensity[j].second);

                    // If the difference is large enough it means it is a new layer
                    if (difference > 0.01)
                    {
                        // Calculate the average layer intensity and store in the temporal_intensities vector
                        double layer_intensity = std::accumulate(temporal_intensities.begin(), temporal_intensities.end(), 0.0) / temporal_intensities.size();
                        layer_intensities.push_back(layer_intensity);

                        // Store the last z coordinate as layer representative
                        layers.push_back(z_intensity[j + 1].first);

                        // Clear the temporal_intensities vector as we are going to process a new layer
                        temporal_intensities.clear();
                    }
                }
                
                // Ensure the cone has at least one layer intensity value before proceeding
                if (layer_intensities.empty()) continue;

                // Divide the layer in three parts with equal height
                size_t n = std::round(layers.size() / 3);

                // Iterate on the center part of the previus division
                for (size_t k = n; k < 2 * n; ++k)
                {
                    // Store the intensities of the center layers
                    middle_intensities.push_back(layer_intensities[k]);
                }

                // Check that the cone enough layers for the divison (validation)
                if (middle_intensities.empty()) continue;

                // Calculate the average layer intensity of the center layers of the cone and store it the averages_id vector
                double average_intensity = std::accumulate(middle_intensities.begin(), middle_intensities.end(), 0.0) / middle_intensities.size();
                averages_id.emplace_back(average_intensity, i);

                std::cout << "Average intensity: " << average_intensity << std::endl;
            }

            // Clear the rest of the vectors as we are going to process a new cone
            z_intensity.clear();
            temporal_intensities.clear();
            layer_intensities.clear();
            layers.clear();
            middle_intensities.clear();
        }

        // Check that the lidar has cones inside the specified range (validation)
        if (averages_id.empty()) return;

        // Calculate the average layer intensity of every cone processed before using a lambda function
        double global_sum = std::accumulate(averages_id.begin(), averages_id.end(), 0.0, [](double acc, const std::pair<double, size_t>& p) 
        {
            return acc + p.first;
        });
        double global_average_intensity = global_sum / averages_id.size();

        std::cout << "Global average: " << global_average_intensity << std::endl;

        // iterate on each (average, id) pair
        for (const auto& pair : averages_id)
        {
            // Define the id for more clarity
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

        // Iterate on each center
        /*for (size_t k = 0; k < clusters_centers.size(); ++k)
        {
            // Iterate on each average intensity - cone id pair
            for (const auto& pair : averages_id)
            {
                // If the cone has one of the stores id we may reassign a new color to it
                size_t id = pair.second;
                if (k == id)
                {
                    // If the cone has a much larger intensity than the global average it means is a left cone
                    if (pair.first > global_average_intensity + coloring_threshold)
                    {
                        clusters_centers[k].color = 1;
                    }
                    // If the cone has a much larger intensity than the global average it means is a right cone
                    else if (pair.first < global_average_intensity - coloring_threshold)
                    {
                        clusters_centers[k].color = 2;
                    }
                }
            }
        }*/
    }
 }