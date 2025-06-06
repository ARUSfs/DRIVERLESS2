/**
 * @file scoring.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 */
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "PointXYZColorScore.h"
 

namespace Utils 
{
    /**
    * @brief Extract the center of each cluster.
    */
    void get_clusters_centers(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* cluster_clouds,
        std::vector<pcl::PointXYZI>* clusters_centers)
    {
        for (auto cluster : *cluster_clouds)
        {
            if (!cluster || cluster->empty()) continue;

            // Compute the centroid of the cluster
            pcl::CentroidPoint<pcl::PointXYZI> centroid_filter;
            for (const auto& point : cluster->points)
            {
                centroid_filter.add(point);
            }
            pcl::PointXYZI centroid;
            centroid_filter.get(centroid);

            // Add the center point to the clusters_centers cloud
            clusters_centers->push_back(centroid);
        }

    }


    
    /**
    * @brief Filter the final clusters by size to delete the ones that are too small or too large to be considered cones.
    */
    void filter_clusters(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>* cluster_clouds,
        std::vector<pcl::PointXYZI>* clusters_centers, double min_height = 0.10, double max_height = 0.4,
        double top_z = 1.0, double max_width = 0.5)
    {
        // Inverse loop to erase safely
        for (int i = (int) cluster_clouds->size() - 1; i >= 0; --i) {
            auto cluster = (*cluster_clouds)[i];
            if (!cluster || cluster->empty()) continue;

            // Obtain the new bounding box of the cluster
            pcl::PointXYZI min_point, max_point;
            pcl::getMinMax3D(*cluster, min_point, max_point);
            double height = max_point.z - min_point.z;
            double width_x = max_point.x - min_point.x;
            double width_y = max_point.y - min_point.y;

            // Filter the cluster by size and keep the center of the cluster
            if (height < min_height || height > max_height || width_x > max_width || width_y > max_width || max_point.z > top_z)
            {
                // Remove the cluster from both vectors
                cluster_clouds->at(i)->clear();
                clusters_centers->erase(clusters_centers->begin() + i);
                cluster_clouds->erase(cluster_clouds->begin() + i);
            }
            else if (height < min_height) // If the height is too small, remove it
            {
                clusters_centers->erase(clusters_centers->begin() + i);
                cluster_clouds->erase(cluster_clouds->begin() + i);

            }
        }
    
        // Resize the vectors
        cluster_clouds->resize(clusters_centers->size());
        clusters_centers->resize(clusters_centers->size());
    }


    /**
    * @brief Recover falsely ground filtered points.
    */
    void reconstruction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, 
        std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore> clusters_centers, 
        double radius)
    {
        for (size_t i = 0; i < clusters_centers.size(); ++i)
        {
            // Convert from PointXYZColorScore to PointXYZI
            pcl::PointXYZI center;
            center.x = clusters_centers[i].x;
            center.y = clusters_centers[i].y;
            center.z = clusters_centers[i].z;
            center.intensity = clusters_centers[i].score;
            
            for (size_t j = 0; j < cloud_plane->size(); ++j)
            {
                pcl::PointXYZI point = cloud_plane->points[j];

                // Check if the point lies inside the cylinder
                double dx = point.x - center.x;
                double dy = point.y - center.y;
                double radial_distance = std::sqrt(dx * dx + dy * dy);

                if (radial_distance <= radius)
                {
                    // Add the point to the filtered cloud
                    cloud_filtered->points.push_back(point);
                    cloud_filtered->width++;

                    // Add the index of the recovered point to cluster indices
                    cluster_indices[i].indices.push_back(cloud_filtered->points.size() - 1);
                }
            }
        }
    }



    /**
    * @brief Correct the motion of the points in the final map.
    */
    void motion_correction(pcl::PointCloud<PointXYZColorScore>::Ptr cloud, double vx, double vy, double yaw_rate, double dt)
    {
        // Estimate the motion of the points
        double theta = -yaw_rate * dt; // Estimate DSK delay

        // Apply the estimated motion to the points
        for (auto& p : cloud->points)
        {
            double dx = vx * dt, dy = vy * dt;
            p.x = p.x * std::cos(theta) - p.y * std::sin(theta) - dx;
            p.y = p.x * std::sin(theta) + p.y * std::cos(theta) - dy;
        }
    } 

    void ground_align(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::ModelCoefficients::Ptr& coefficients) {

        if (coefficients->values.size() != 4) {
            std::cout << "Invalid coefficients size: " << coefficients->values.size() << std::endl;
            return;
        }

        Eigen::Vector3f plane_normal(coefficients->values[0],
                                    coefficients->values[1],
                                    coefficients->values[2]);
        plane_normal.normalize();

        Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(plane_normal, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();

        float d = coefficients->values[3];
        Eigen::Vector3f point_on_plane = -d * plane_normal;

        Eigen::Vector3f rotated_point = rotation_matrix * point_on_plane;

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = rotation_matrix;
        transform.block<3,1>(0,3) = -rotated_point;

        pcl::transformPointCloud(*cloud, *cloud, transform);
    }
}