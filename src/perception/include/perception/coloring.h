/**
 * @file statistical_coloring.h
 * @author Rafael Guil Valero (rafaguilvalero@gmail.com)
 * @brief Contains auxiliar functions used in the algorithm to estimate the color of the closest cones.
*/

#include "PointXYZProbColorScore.h"
#include <algorithm>
#include <boost/math/distributions/normal.hpp>
#include <fstream>


namespace Coloring
{
    /**
    * @brief Estimates the color of detected cones based on intensity. 
    */
    void color_estimation(pcl::PointCloud<PointXYZProbColorScore>::Ptr final_map, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_points, std::vector<pcl::PointXYZI>& clusters_centers,
        double distance_threshold, bool kDebug)
    {
        double ayellow = -0.33;
        double ablue   = -0.5;
        double byellow = 13;
        double bblue   = 26;

        double sigma_blue = 2.5;
        double sigma_yellow = 2.5;

        double r, avg_yellow_intensity, avg_blue_intensity;

        for(size_t i = 0; i < clusters_centers.size(); i++)
        {
            if(std::sqrt(std::pow(clusters_centers[i].x, 2) + std::pow(clusters_centers[i].y, 2)) > distance_threshold
                || std::sqrt(std::pow(clusters_centers[i].x, 2) + std::pow(clusters_centers[i].y, 2)) < 0.0 ||
                clusters_centers[i].x < 0 || cluster_points[i]->points.size() < 10)
            {
                final_map->points[i].prob_yellow = 0.0; // unknown
                final_map->points[i].prob_blue   = 0.0; // unknown
                continue; 
            }
            
            r = std::sqrt(std::pow(clusters_centers[i].x, 2) + std::pow(clusters_centers[i].y, 2));
            double p_r = std::exp(-0.015 * r);                  // Exponential decay
            avg_yellow_intensity = ayellow * r + byellow;       // Linear function for expected yellow intensity
            avg_blue_intensity   = ablue   * r + bblue;         // Linear function for expected blue intensity
            


            // Yellow probability
            // P(yellow cone) = P(i <= avg_intensity + sigma^2) - P(i<=avg_intensity - sigma^2)
            double mean_yellow = avg_yellow_intensity;
            boost::math::normal dist_yellow(mean_yellow, sigma_yellow);
            double prob_yellow = boost::math::cdf(dist_yellow, clusters_centers[i].intensity + sigma_yellow*sigma_yellow) 
                                - boost::math::cdf(dist_yellow, clusters_centers[i].intensity - sigma_yellow*sigma_yellow);

            // Blue probability
            // P(blue cone)   = P(i <= avg_intensity + sigma^2) - P(i<=avg_intensity - sigma^2)
            double mean_blue = avg_blue_intensity;
            boost::math::normal dist_blue(mean_blue, sigma_blue);
            double prob_blue   = boost::math::cdf(dist_blue, clusters_centers[i].intensity+ sigma_blue*sigma_blue) 
                                - boost::math::cdf(dist_blue, clusters_centers[i].intensity - sigma_blue*sigma_blue);

            final_map->points[i].prob_yellow = prob_yellow * p_r;
            final_map->points[i].prob_blue   = prob_blue   * p_r;

            
        }
    }
}