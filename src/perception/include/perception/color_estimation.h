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
    void color_estimation1(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        double distance_threshold = 5.0;
        std::vector<std::pair<double, size_t>> a_id_pairs;
        double cte = 250.0;

        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < distance_threshold)
            {
                std::cout << "Distancia: " << distance << std::endl;
                
                std::vector<std::pair<double, double>> z_intensity_pairs;
                std::vector<double> z_values, intensities, intensitiesInLayer, layerIntensities, layers;
                
                for (const auto& index : indices.indices) 
                {
                    const auto& point = cloud_filtered->points[index];
                    z_intensity_pairs.emplace_back(point.z, point.intensity);
                }

                std::sort(z_intensity_pairs.begin(), z_intensity_pairs.end(), [](const std::pair<double, double>& a, const std::pair<double, double>& b) 
                {
                    return a.first < b.first;
                });

                for (const auto& pair : z_intensity_pairs) 
                {
                    z_values.push_back(pair.first);
                    intensities.push_back(pair.second);
                }

                for (size_t j = 0; j < z_values.size() - 1; ++j)
                {
                    double difference = z_values[j + 1] - z_values[j];
                    intensitiesInLayer.push_back(intensities[j]);

                    if (difference > 0.01)
                    {
                        double inten = std::accumulate(intensitiesInLayer.begin(), intensitiesInLayer.end(), 0.0) / intensitiesInLayer.size();
                        layerIntensities.push_back(inten);
                        intensitiesInLayer.clear();

                        layers.push_back(z_values[j + 1]);
                    }
                }
                
                if (layerIntensities.size() == 0) continue;

                size_t n = layers.size();

                Eigen::MatrixXd A(n, 3);
                Eigen::VectorXd b(n);

                for (size_t k = 0; k < n; ++k) 
                {
                    double x = layers[k];
                    double y = layerIntensities[k];
                    A(k, 0) = x * x;
                    A(k, 1) = x;
                    A(k, 2) = 1.0;
                    b(k) = y;
                }

                Eigen::VectorXd coefficients = A.colPivHouseholderQr().solve(b);
                double a = std::abs(coefficients(0));
                a_id_pairs.emplace_back(a, i);

                std::cout << "A " << a << std::endl;
            }
        }

        double sum = std::accumulate(a_id_pairs.begin(), a_id_pairs.end(), 0.0, [](double acc, const std::pair<double, size_t>& p) 
        {
            return acc + p.first;
        });
        double globalA = sum / a_id_pairs.size();
        std::cout << "Global a: " << globalA << std::endl;

        for (size_t k = 0; k < clusters_centers.size(); ++k)
        {
            for (const auto& pair : a_id_pairs)
            {
                size_t id = pair.second;
                if (k == id)
                {
                    if (pair.first > globalA + cte)
                    {
                        clusters_centers[k].color = 1;
                    }
                    else if (pair.first < globalA - cte)
                    {
                        clusters_centers[k].color = 2;
                    }
                }
            }
        }
    }

    void color_estimation2(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        double distance_threshold = 5.0;
        std::vector<std::pair<double, size_t>> averages_id_pairs;
        double cte = 15.0;

        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < distance_threshold)
            {
                std::cout << "Distancia: " << distance << std::endl;

                std::vector<std::pair<double, double>> z_intensity_pairs;
                std::vector<double> z_values, intensities, intensitiesInLayer, layerIntensities, layers, middle_i;
                
                for (const auto& index : indices.indices) 
                {
                    const auto& point = cloud_filtered->points[index];
                    z_intensity_pairs.emplace_back(point.z, point.intensity);
                }

                std::sort(z_intensity_pairs.begin(), z_intensity_pairs.end(), [](const std::pair<double, double>& a, const std::pair<double, double>& b) 
                {
                    return a.first < b.first;
                });

                for (const auto& pair : z_intensity_pairs) 
                {
                    z_values.push_back(pair.first);
                    intensities.push_back(pair.second);
                }

                for (size_t j = 0; j < z_values.size() - 1; ++j)
                {
                    double difference = z_values[j + 1] - z_values[j];
                    intensitiesInLayer.push_back(intensities[j]);

                    if (difference > 0.01)
                    {
                        double inten = std::accumulate(intensitiesInLayer.begin(), intensitiesInLayer.end(), 0.0) / intensitiesInLayer.size();
                        layerIntensities.push_back(inten);
                        intensitiesInLayer.clear();

                        layers.push_back(z_values[j + 1]);
                    }
                }
                
                if (layerIntensities.size() == 0) continue;

                size_t n = std::round(layers.size() / 3);

                for (size_t k = n; k < 2 * n; ++k)
                {
                    middle_i.push_back(layerIntensities[k]);
                }

                if (middle_i.size() == 0) continue;

                double average = std::accumulate(middle_i.begin(), middle_i.end(), 0.0) / middle_i.size();
                averages_id_pairs.emplace_back(average, i);
                std::cout << "Average: " << average << std::endl;
            }
        }

        double sum = std::accumulate(averages_id_pairs.begin(), averages_id_pairs.end(), 0.0, [](double acc, const std::pair<double, size_t>& p) 
        {
            return acc + p.first;
        });
        double globalAvg = sum / averages_id_pairs.size();
        std::cout << "Global avg: " << globalAvg << std::endl;

        for (size_t k = 0; k < clusters_centers.size(); ++k)
        {
            for (const auto& pair : averages_id_pairs)
            {
                size_t id = pair.second;
                if (k == id)
                {
                    if (pair.first > globalAvg + cte)
                    {
                        clusters_centers[k].color = 1;
                    }
                    else if (pair.first < globalAvg - cte)
                    {
                        clusters_centers[k].color = 2;
                    }
                }
            }
        }
    }
 }