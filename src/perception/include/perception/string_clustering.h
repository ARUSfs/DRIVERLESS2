#include <random>
/**
 * @brief Clase que implementa el algoritmo String Clustering para segmentación de nubes de puntos LiDAR.
 */
class StringClustering
{
public:
    struct Cluster
    {
        std::vector<int> indices; 
        Eigen::Vector3f centroid;
    };
    
    std::vector<Cluster> clusters;
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud; 

    double max_arc_length = 0.285;  
    double max_radial_distance = 0.285; 

    void clusterPoints(const pcl::PointCloud<PointXYZIRingTime>::Ptr &cloud)
    {
        clusters.clear();

        for (size_t i = 0; i < cloud->size(); i++)
        {
            const auto &point = cloud->points[i];
            Eigen::Vector3f point_pos(point.x, point.y, point.z);

            bool added = false;
            for (auto &cluster : clusters)
            {
                Eigen::Vector3f cluster_pos = cluster.centroid;

                // Calcular distancias radial y en arco
                double radial_distance = (point_pos - cluster_pos).norm();
                double arc_length_distance = std::fabs(atan2(point.y, point.x) - atan2(cluster_pos.y(), cluster_pos.x())) * std::sqrt(point.x * point.x + point.y * point.y);

                double max_distance = std::max(radial_distance, arc_length_distance);

                // Si el punto está lo suficientemente cerca, añadir al cluster
                if (max_distance < max_arc_length)
                {
                    cluster.indices.push_back(i);
                    cluster.centroid = (cluster.centroid * (cluster.indices.size() - 1) + point_pos) / cluster.indices.size();
                    added = true;
                    break;
                }
            }

            // Si el punto no se añadió a ningún cluster, crear uno nuevo
            if (!added)
            {
                Cluster new_cluster;
                new_cluster.indices.push_back(i);
                new_cluster.centroid = point_pos;
                clusters.push_back(new_cluster);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr getClusteredPointCloud()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // Generador de colores aleatorios (intensidad entre 20 y 255)
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> intensity_dist(20.0, 255.0);

        std::vector<float> cluster_colors;
        cluster_colors.reserve(clusters.size());

        // Asignamos un color único a cada cluster
        for (size_t i = 0; i < clusters.size(); i++)
        {
            cluster_colors.push_back(intensity_dist(rng));
        }

        // Asignamos la intensidad (color) a los puntos de cada cluster
        for (size_t i = 0; i < clusters.size(); i++)
        {
            float cluster_color = cluster_colors[i]; // Color único para este cluster
            for (const int idx : clusters[i].indices)
            {
                pcl::PointXYZI p = original_cloud->points[idx]; // Extraemos el punto original
                p.intensity = cluster_color;  // Asignamos la nueva intensidad/color
                clustered_cloud->points.push_back(p);
            }
        }

        clustered_cloud->width = clustered_cloud->points.size();
        clustered_cloud->height = 1;
        clustered_cloud->is_dense = true;

        return clustered_cloud;
    }
};