/**
 * @file kmeans_coloring.h
 * @author Rafael Guil Valero (rafaguilvalero@gmail.com)
 * @brief Contains auxiliar functions used in the algorithm to estimate the color of the closest cones.
*/

#include "PointXYZColorScore.h"
#include <algorithm>
#include <pcl/ml/kmeans.h>
#include <boost/math/distributions/normal.hpp>


namespace KMeansColoring
{
    /**
    * @brief Estimates the color of detected cones based on intensity. 
    */
    void color_estimation(std::vector<pcl::PointCloud<PointXYZIRingTime>::Ptr> cluster_points, std::vector<PointXYZColorScore>& clusters_centers,
        double distance_threshold)
    {
        std::vector<std::vector<float>> intensity_points;
        for(size_t i = 0; i < cluster_points.size(); i++)
        {
            for(const auto& pt : cluster_points[i]->points)
            {
                intensity_points.push_back({pt.intensity});
            }
        }

        int k = 2; // Number of clusters (yellow, blue)
        pcl::Kmeans kmeans(intensity_points.size(), 1);
        kmeans.setClusterSize(k);
        kmeans.setInputData(intensity_points);
        kmeans.kMeans();
        auto centroids = kmeans.get_centroids();
        if (centroids.size() < k) {
            std::cout << "Not enough centroids." << std::endl;
            return;
        }

        double avg_yellow_intensity = std::min(centroids[0][0], centroids[1][0]);
        double avg_blue_intensity   = (centroids[0][0] + centroids[1][0]) / 2.0; // Average of the two centroids
        std::cout << "Average blue intensity: "  << avg_blue_intensity  << std::endl;
        std::cout << "Average yellow intensity: " << avg_yellow_intensity << std::endl;

        for(size_t i = 0; i < clusters_centers.size(); i++)
        {
            if(std::sqrt(std::pow(clusters_centers[i].x, 2) + std::pow(clusters_centers[i].y, 2)) > distance_threshold ||
                std::sqrt(std::pow(clusters_centers[i].x, 2) + std::pow(clusters_centers[i].y, 2)) < 3.0)
            {
                clusters_centers[i].prob_yellow = -1; // unknown
                clusters_centers[i].prob_blue   = -1; // unknown
                continue; // Skip points that are too far from the cluster center
            }

            double sum_intensity = 0.0;
            for(const auto& pt : cluster_points[i]->points)
            {                
                sum_intensity += pt.intensity; // Accumulate intensity score
            }
            int count_intensity = cluster_points[i]->points.size();            
            if(count_intensity > 0 && !std::isnan(avg_blue_intensity) && !std::isnan(avg_yellow_intensity)) 
            {
                double avg_cluster_intensity = sum_intensity / count_intensity;

                // Yellow probability
                double mean_yellow = avg_yellow_intensity;
                double sigma_yellow = 5.0;
                boost::math::normal dist_yellow(mean_yellow, sigma_yellow);
                double pdf_yellow = pdf(dist_yellow, avg_cluster_intensity);

                // Blue probability
                double mean_blue = avg_blue_intensity;
                double sigma_blue = 5.0;
                boost::math::normal dist_blue(mean_blue, sigma_blue);
                double pdf_blue   = pdf(dist_blue, avg_cluster_intensity);

                double prior_yellow = 0.5;
                double prior_blue = 0.5;
                if (avg_cluster_intensity < avg_yellow_intensity) {
                    clusters_centers[i].prob_yellow = 1.0; // More likely to be yellow
                    clusters_centers[i].prob_blue = 0.0;   // Less likely to be blue
                } else if (avg_cluster_intensity > avg_blue_intensity) {
                    clusters_centers[i].prob_yellow = 0.0; // Less likely to be yellow
                    clusters_centers[i].prob_blue = 1.0;   // More likely to be blue
                } else {
                    double Pyellow = pdf_yellow * prior_yellow / (pdf_yellow * prior_yellow + pdf_blue * prior_blue);
                    double Pblue   = pdf_blue   * prior_blue   / (pdf_yellow * prior_yellow + pdf_blue * prior_blue);
                    clusters_centers[i].prob_yellow = Pyellow;
                    clusters_centers[i].prob_blue   = Pblue;
                }

                std::cout << "x: " << clusters_centers[i].x << "; y: " << clusters_centers[i].y << "; prob_yellow = " << clusters_centers[i].prob_yellow
                << ", prob_blue = " << clusters_centers[i].prob_blue << "; avg_cluster_intensity:" << avg_cluster_intensity << std::endl;
            }
        }
    }
}