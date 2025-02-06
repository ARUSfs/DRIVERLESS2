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
    void color_estimation(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        double distance_threshold = 3.0;

        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < distance_threshold)
            {
                std::cout << "Distancia: " << distance << std::endl;
                
                std::vector<double> z_values, intensities, avgIntensities, intensitiesInLayer;
                
                for (const auto& index : indices.indices) 
                {
                    const auto& point = cloud_filtered->points[index];
                    z_values.push_back(point.z);
                    intensities.push_back(point.intensity);
                }

                std::sort(z_values.begin(), z_values.end());

                for (size_t j = 0; j < z_values.size() - 1; ++j)
                {
                    double difference = z_values[j + 1] - z_values[j];
                    intensitiesInLayer.push_back(intensities[j]);

                    if (difference > 0.01)
                    {
                        double avgIntensity = std::accumulate(intensitiesInLayer.begin(), intensitiesInLayer.end(), 0.0) / intensitiesInLayer.size();
                        avgIntensities.push_back(avgIntensity);
                        intensitiesInLayer.clear();

                        std::cout << "Layer intensity: " << avgIntensity << std::endl;
                    }
                }
                
                if (avgIntensities.size() == 0) continue;

                double minI = *std::min_element(avgIntensities.begin(), avgIntensities.end());
                double maxI = *std::max_element(avgIntensities.begin(), avgIntensities.end());
                double globalIntensity = maxI - minI;
                double avgGlobalIntensity = std::accumulate(avgIntensities.begin(), avgIntensities.end(), 0.0) / avgIntensities.size();
                double deviation = avgGlobalIntensity - globalIntensity;

                std::cout << "Intensity difference: " << globalIntensity << std::endl;
                std::cout << "Deviation: " << avgGlobalIntensity - globalIntensity << std::endl;

                if (deviation < -10)
                {
                    center.color = 4;

                    std::cout << "Color cambiado a: " << center.color << std::endl;    
                }
                else if (deviation > 10)
                {
                    center.color = 5;

                    std::cout << "Color cambiado a: " << center.color << std::endl;    
                }
            }
        }
    }

    void color_estimation2(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        double distance_threshold = 3.0;

        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < distance_threshold)
            {
                std::cout << "Distancia: " << distance << std::endl;
                
                std::vector<double> z_values, intensities, avgIntensities, intensitiesInLayer, layers;
                
                for (const auto& index : indices.indices) 
                {
                    const auto& point = cloud_filtered->points[index];
                    z_values.push_back(point.z);
                    intensities.push_back(point.intensity);
                }

                std::sort(z_values.begin(), z_values.end());

                for (size_t j = 0; j < z_values.size() - 1; ++j)
                {
                    double difference = z_values[j + 1] - z_values[j];
                    intensitiesInLayer.push_back(intensities[j]);

                    if (difference > 0.01)
                    {
                        double avgIntensity = std::accumulate(intensitiesInLayer.begin(), intensitiesInLayer.end(), 0.0) / intensitiesInLayer.size();
                        avgIntensities.push_back(avgIntensity);
                        intensitiesInLayer.clear();

                        layers.push_back(z_values[j + 1]);

                        std::cout << "Layer intensity: " << avgIntensity << std::endl;
                    }
                }
                
                if (avgIntensities.size() == 0) continue;

                double avg = std::accumulate(intensities.begin(), intensities.end(), 0.0) / intensities.size();

                std::cout << "Average intensity: " << avg << std::endl;

                /*size_t n = layers.size();
                Eigen::MatrixXd A(n, 3);
                Eigen::VectorXd b(n);

                for (size_t k = 0; k < n; ++k) 
                {
                    double x = layers[k];
                    double y = avgIntensities[k];
                    A(k, 0) = x * x;
                    A(k, 1) = x;
                    A(k, 2) = 1.0;
                    b(k) = y;
                }

                Eigen::VectorXd coefficients = A.colPivHouseholderQr().solve(b);
                double a = std::abs(coefficients(0));

                if (a > 1000)
                {
                    center.color = 1;
                }
                else if (a < 500)
                {
                    center.color = 2;
                }

                std::cout << "A " << coefficients(0) << std::endl;*/
                
                center.color = 1;
            }
        }
    }

    void color_estimation3(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        double distance_threshold = 3.0;

        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < distance_threshold)
            {
                std::cout << "Distancia: " << distance << std::endl;
                
                std::vector<double> z_values, intensities, avgIntensities, intensitiesInLayer, layers;
                
                for (const auto& index : indices.indices) 
                {
                    const auto& point = cloud_filtered->points[index];
                    z_values.push_back(point.z);
                    intensities.push_back(point.intensity);
                }

                std::sort(z_values.begin(), z_values.end());

                for (size_t j = 0; j < z_values.size() - 1; ++j)
                {
                    double difference = z_values[j + 1] - z_values[j];
                    intensitiesInLayer.push_back(intensities[j]);

                    if (difference > 0.01)
                    {
                        double avgIntensity = std::accumulate(intensitiesInLayer.begin(), intensitiesInLayer.end(), 0.0) / intensitiesInLayer.size();
                        avgIntensities.push_back(avgIntensity);
                        intensitiesInLayer.clear();

                        layers.push_back(z_values[j + 1]);

                        std::cout << "Layer intensity: " << avgIntensity << std::endl;
                    }
                }
                
                if (avgIntensities.size() == 0) continue;

                size_t n = layers.size();

                double avgGlobalIntensity = std::accumulate(avgIntensities.begin(), avgIntensities.end(), 0.0) / avgIntensities.size();

                for (size_t k = 0; k < n; ++k)
                {
                    if (avgIntensities[k] < 0.30 * avgGlobalIntensity)
                    {
                        avgIntensities.erase(avgIntensities.begin() + k);
                        layers.erase(layers.begin() + k);
                    }
                }

                double minI = *std::min_element(avgIntensities.begin(), avgIntensities.end());
                double maxI = *std::max_element(avgIntensities.begin(), avgIntensities.end());
                double globalIntensity = maxI - minI;
                double new_avgGlobalIntensity = std::accumulate(avgIntensities.begin(), avgIntensities.end(), 0.0) / avgIntensities.size();
                double deviation = std::abs(new_avgGlobalIntensity - globalIntensity);
                
                for (double i : avgIntensities)
                {
                    std::cout << "Layer intesity after: " << i << std::endl;
                }

                std::cout << "Deviation: " << deviation << std::endl;

                Eigen::MatrixXd A(n, 3);
                Eigen::VectorXd b(n);

                for (size_t k = 0; k < n; ++k) 
                {
                    double x = layers[k];
                    double y = avgIntensities[k];
                    A(k, 0) = x * x;
                    A(k, 1) = x;
                    A(k, 2) = 1.0;
                    b(k) = y;
                }

                Eigen::VectorXd coefficients = A.colPivHouseholderQr().solve(b);
                double a = std::abs(coefficients(0));

                std::cout << "A " << coefficients(0) << std::endl;

                if (deviation < 15 && a < 500)
                {
                    center.color = 1; 
                }
                else if (deviation > 15 && a > 1000)
                {
                    center.color = 2; 
                }   
            }
        }
    }

    void color_estimation4(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        double distance_threshold = 3.0;
        std::vector<double> avgs;
        double coloring_threshold = 8.0;

        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < distance_threshold)
            {
                std::cout << "Distancia: " << distance << std::endl;
                
                std::vector<double> intensities;
                
                for (const auto& index : indices.indices) 
                {
                    const auto& point = cloud_filtered->points[index];
                    intensities.push_back(point.intensity);
                }

                double avg = std::accumulate(intensities.begin(), intensities.end(), 0.0) / intensities.size();
                avgs.push_back(avg);

                intensities.clear();

                std::cout << "Average intensity: " << avg << std::endl;
            }
        }

        double global_avg = std::accumulate(avgs.begin(), avgs.end(), 0.0) / avgs.size();

        std::cout << "Global average intensity: " << global_avg << std::endl;

        for (size_t k = 0; k < avgs.size(); ++k)
        {
            if (avgs[k] > global_avg + coloring_threshold)
            {
                clusters_centers[k].color = 1;
            }
            else if (avgs[k] < global_avg - coloring_threshold)
            {
                clusters_centers[k].color = 2;
            }
        }
    }

    void color_estimation5(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        double distance_threshold = 3.0;
        std::vector<double> avgLayers;
        double coloring_threshold = 8.0;

        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < distance_threshold)
            {
                std::cout << "Distancia: " << distance << std::endl;
                
                std::vector<double> z_values, intensities, avgs, intensitiesInLayer;
                
                for (const auto& index : indices.indices) 
                {
                    const auto& point = cloud_filtered->points[index];
                    z_values.push_back(point.z);
                    intensities.push_back(point.intensity);
                }

                std::sort(z_values.begin(), z_values.end());

                for (size_t j = 0; j < z_values.size() - 1; ++j)
                {
                    double difference = z_values[j + 1] - z_values[j];
                    intensitiesInLayer.push_back(intensities[j]);

                    if (difference > 0.01)
                    {
                        double avgIntensity = std::accumulate(intensitiesInLayer.begin(), intensitiesInLayer.end(), 0.0) / intensitiesInLayer.size();
                        avgs.push_back(avgIntensity);
                        intensitiesInLayer.clear();
                    }
                }

                double avgLayer = std::accumulate(avgs.begin(), avgs.end(), 0.0) / avgs.size();
                avgLayers.push_back(avgLayer);

                std::cout << "Avg layer intensity: " << avgLayer << std::endl;
            }
        }

        double global_avg = std::accumulate(avgLayers.begin(), avgLayers.end(), 0.0) / avgLayers.size();

        std::cout << "Global layer intensity: " << global_avg << std::endl;

        for (size_t k = 0; k < avgLayers.size(); ++k)
        {
            if (avgLayers[k] > global_avg + coloring_threshold)
            {
                clusters_centers[k].color = 1;
            }
            else if (avgLayers[k] < global_avg - coloring_threshold)
            {
                clusters_centers[k].color = 2;
            }
        }
    }

    void color_estimation45(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        double distance_threshold = 3.0;
        std::vector<double> avgLayers, avgss;
        double coloring_threshold = 8.0;

        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < distance_threshold)
            {
                std::cout << "Distancia: " << distance << std::endl;
                
                std::vector<double> z_values, intensities, avgs, intensitiesInLayer;
                
                for (const auto& index : indices.indices) 
                {
                    const auto& point = cloud_filtered->points[index];
                    z_values.push_back(point.z);
                    intensities.push_back(point.intensity);
                }

                double avgg = std::accumulate(intensities.begin(), intensities.end(), 0.0) / intensities.size();
                avgss.push_back(avgg);

                intensities.clear();

                std::cout << "Average intensity: " << avgg << std::endl;

                std::sort(z_values.begin(), z_values.end());

                for (size_t j = 0; j < z_values.size() - 1; ++j)
                {
                    double difference = z_values[j + 1] - z_values[j];
                    intensitiesInLayer.push_back(intensities[j]);

                    if (difference > 0.01)
                    {
                        double avgIntensity = std::accumulate(intensitiesInLayer.begin(), intensitiesInLayer.end(), 0.0) / intensitiesInLayer.size();
                        avgs.push_back(avgIntensity);
                        intensitiesInLayer.clear();
                    }
                }

                double avgLayer = std::accumulate(avgs.begin(), avgs.end(), 0.0) / avgs.size();
                avgLayers.push_back(avgLayer);

                std::cout << "Avg layer intensity: " << avgLayer << std::endl;
            }
        }

        double global_avg = std::accumulate(avgLayers.begin(), avgLayers.end(), 0.0) / avgLayers.size();

        std::cout << "Global layer intensity: " << global_avg << std::endl;

        double global_avgg = std::accumulate(avgss.begin(), avgss.end(), 0.0) / avgss.size();

        std::cout << "Global average intensity: " << global_avgg << std::endl;

        for (size_t k = 0; k < avgLayers.size(); ++k)
        {
            if (avgLayers[k] > global_avg + coloring_threshold || avgss[k] > global_avgg + coloring_threshold)
            {
                clusters_centers[k].color = 1;
            }
            else if (avgLayers[k] < global_avg - coloring_threshold || avgss[k] < global_avgg - coloring_threshold)
            {
                clusters_centers[k].color = 2;
            }
        }
    }
 }