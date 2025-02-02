/**
 * @file color_estimation.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to estimate the color of the closest cones.
 * @version 0.1
 * @date 20-1-2025
 */

namespace ColorEstimation
{
    void coloring(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < 3)
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

                /*double minZ = *std::min_element(z_values.begin(), z_values.end());
                double maxZ = *std::max_element(z_values.begin(), z_values.end());

                std::sort(z_values.begin(), z_values.end());

                std::vector<double> dz_;
                for (size_t l = 0; l < z_values.size() - 1; ++l)
                {
                    dz_.push_back(z_values[l + 1] - z_values[l]);
                }

                double minThreshold = 0.01;
                std::vector<double> dz;
                for (double d : dz_) 
                {
                    if (d > minThreshold) dz.push_back(d);
                }

                layerHeight = dz.empty() ? 0.01 : std::accumulate(dz.begin(), dz.end(), 0.0) / dz.size();

                std::vector<double> layers, avgIntensities;

                for (double z = minZ; z <= maxZ; z += layerHeight)
                {
                    std::vector<double> intensitiesInLayer;

                    for (size_t j = 0; j < z_values.size(); ++j)
                    {
                        if (z_values[j] >= z && z_values[j] < z + layerHeight)
                        {
                            intensitiesInLayer.push_back(intensities[j]);
                        }
                    }
                    
                    if(intensitiesInLayer.size() > 0)
                    {
                        layers.push_back(z);

                        double avgIntensity = std::accumulate(intensitiesInLayer.begin(), intensitiesInLayer.end(), 0.0) / intensitiesInLayer.size();
                        avgIntensities.push_back(avgIntensity);

                        std::cout << "Layer intensity: " << avgIntensity << std::endl;
                    }
                }*/

                /*double minI = *std::min_element(avgIntensities.begin(), avgIntensities.end());
                double maxI = *std::max_element(avgIntensities.begin(), avgIntensities.end());
                double avgGlobalIntensity = std::accumulate(avgIntensities.begin(), avgIntensities.end(), 0.0) / avgIntensities.size();
                double globalIntensity = maxI - minI;

                std::cout << "Max intensity: " << maxI << std::endl;
                std::cout << "Min intensity: " << minI << std::endl;
                std::cout << "Intensity difference: " << globalIntensity << std::endl;
                std::cout << "Deviation: " << avgGlobalIntensity - globalIntensity << std::endl;*/

                /*size_t n = layers.size();
                Eigen::VectorXd x(n);
                Eigen::VectorXd y(n);

                for (size_t k = 0; k < n; ++k) 
                {
                    x[k] = k;
                    y[k] = avgIntensities[k];
                }

                Eigen::MatrixXd A(n, 2);
                A.col(0) = x;
                A.col(1) = Eigen::VectorXd::Ones(n);
                Eigen::VectorXd coeffs_line = A.colPivHouseholderQr().solve(y);
                Eigen::VectorXd y_line_fit = A * coeffs_line;
                double linear_error = (y - y_line_fit).squaredNorm() / n;

                Eigen::MatrixXd B(n, 3);
                B.col(0) = x.array().square();
                B.col(1) = x;
                B.col(2) = Eigen::VectorXd::Ones(n);
                Eigen::VectorXd coeffs_quad = B.colPivHouseholderQr().solve(y);
                Eigen::VectorXd y_quad_fit = B * coeffs_quad;
                double quadratic_error = (y - y_quad_fit).squaredNorm() / n;

                std::cout << "Error lineal: " << linear_error << std::endl;
                std::cout << "Error cuadrático: " << quadratic_error << std::endl;

                if (linear_error < 400)
                {
                    center.color = 4;
                }
                else if (linear_error > 700)
                {
                    center.color = 5;
                }*/

                center.color = 4;
            }
        }
    }

    void coloring2(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);
            std::cout << "Distancia: " << distance << std::endl;

            if (distance < 3)
            {
                std::vector<double> z_values, intensities;
                
                for (const auto& index : indices.indices) 
                {
                    const auto& point = cloud_filtered->points[index];
                    z_values.push_back(point.z);
                    intensities.push_back(point.intensity);

                    std::cout << "Z: " << point.z << std::endl;
                    std::cout << "Intensisy: " << point.intensity << std::endl;
                }

                /*double minZ = *std::min_element(z_values.begin(), z_values.end());
                double maxZ = *std::max_element(z_values.begin(), z_values.end());
                double totalHeight = maxZ - minZ;
            
                std::cout << "Minumum z: " << minZ << std::endl;
                std::cout << "Maximum z: " << maxZ << std::endl;
                std::cout << "Rango: " << minZ + totalHeight / 3 << "/" << maxZ - totalHeight / 3 << std::endl;

                for (size_t j = 0; j < z_values.size(); ++j)
                {
                    double z = z_values[j];
                    double intensity = intensities[j];

                    if (!(z >= minZ + totalHeight / 3 && z <= maxZ - totalHeight / 3))
                    {
                        intensities.erase(std::remove(intensities.begin(), intensities.end(), intensity), intensities.end());
                    }
                }*/

                double globalIntensity = std::accumulate(intensities.begin(), intensities.end(), 0.0) / intensities.size();

                std::cout << "Global intensity: " << globalIntensity << std::endl;

                center.color = 4;
            }
        }
    }

    void coloring3(std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore>& clusters_centers, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
    {
        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            PointXYZColorScore& center = clusters_centers[i];
            const auto& indices = cluster_indices[i];

            double distance = std::sqrt(center.x * center.x + center.y * center.y + center.z * center.z);

            if (distance < 3)
            {
                std::cout << "Distancia: " << distance << std::endl;
                
                std::vector<double> z_values, intensities, avgIntensities, intensitiesInLayer, layer;
                
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
                        layer.push_back(z_values[j]);
                        double avgIntensity = std::accumulate(intensitiesInLayer.begin(), intensitiesInLayer.end(), 0.0) / intensitiesInLayer.size();
                        avgIntensities.push_back(avgIntensity);
                        intensitiesInLayer.clear();

                        std::cout << "Layer intensity: " << avgIntensity << std::endl;
                    }
                }

                size_t n = layer.size();
                Eigen::MatrixXd A(n, 3);
                Eigen::VectorXd b(n);

                for (size_t k = 0; k < n; ++k) 
                {
                    double x = layer[k];
                    double y = avgIntensities[k];
                    A(k, 0) = x * x;
                    A(k, 1) = x;
                    A(k, 2) = 1.0;
                    b(k) = y;
                }

                Eigen::VectorXd coefficients = A.colPivHouseholderQr().solve(b);
                coefficients(0) > 0 ? center.color = 1 : center.color = 2;

                std::cout << "A " << coefficients(0) << std::endl;
            }
        }
    }
}