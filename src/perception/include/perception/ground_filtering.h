/**
 * @file ground_filtering.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to remove the ground from the original point cloud. It is 
 * a method that consists of dividing the point cloud into a grid and applying ransac to each square, verifying 
 * that the filtered points correspond to the ground using the normal vectors of the planes.
 * @version 0.1
 * @date 12-11-2024
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/crop_box.h>

namespace GroundFiltering
{
    /**
    * @brief Implements ground filtering using ransac segmentation.
    * @param cloud The raw point cloud that will be filtered.
    * @param cloud_filtered The point cloud that will store the not planar points.
    * @param cloud_plane The point cloud that will store the planar points.
    * @param coefficients The coefficients of the planar ecuation.
    * @param threshold The threshold that will determine if the point belong to the ground.
    */
    void ransac_ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_plane, pcl::ModelCoefficients::Ptr& coefficients, double threshold)
    {   
        //Define the parameters
        pcl::SACSegmentation<pcl::PointXYZI> segmentation;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::PointIndices::Ptr outliers(new pcl::PointIndices);

        //Configure the algorithm
        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(50);
        segmentation.setDistanceThreshold(threshold);

        //Aply the segmentation
        segmentation.setInputCloud(cloud);
        segmentation.segment(*inliers, *coefficients);

        //Verify the segmentation is not empty
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        }

        //Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);

        //Store the not planar points
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
        extract.filter(outliers->indices);

        //Store the planar points
        extract.setNegative(false);
        extract.filter(*cloud_plane);
    }

    /**
    * @brief Auxiliar function for  the function grid_ground_filter, apply ransac in the square specified and check if the result plane is in
    * accordance with the previus ground plane by measuring the angle between their normal vectors, repeating the process until it succeed.
    * @param grid_cloud The raw point cloud of the specified square.
    * @param temp_filtered The temporal cloud cloud that will store the not planar points.
    * @param temp_plane The temporal cloud that will store the planar points.
    * @param coefficients The coefficients of the planar ecuation.
    * @param threshold The threshold that will determine if the point belong to the ground.
    * @param cloud_filtered The point cloud that will store the not planar points.
    * @param cloud_plane The point cloud that will store the planar points.
    * @param prev_normal The normal vector of the previus calculated plane.
    * @param normal The normal vector of the current plane, that will be recalculated.
    * @param angle_threshold The threshold that will determine if the potencial plane is discard as plane or not.
    */
    void ransac_checking_normal_vectors(pcl::PointCloud<pcl::PointXYZI>::Ptr& grid_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& temp_filtered, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr& temp_plane, pcl::ModelCoefficients::Ptr& coefficients, double threshold, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_plane, Eigen::Vector3d& prev_normal, 
        Eigen::Vector3d& normal, double angle_threshold, int minimum_ransac_points)
    {
        // Filter the squares with just a few points
        if (!(grid_cloud->size() < static_cast<std::size_t>(minimum_ransac_points)))
        {
            // Create new temporal clouds to store the ground and not ground points if neccesary
            pcl::PointCloud<pcl::PointXYZI>::Ptr new_temp_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr new_temp_plane(new pcl::PointCloud<pcl::PointXYZI>);

            // Apply ransac
            ransac_ground_filter(grid_cloud, temp_filtered, temp_plane, coefficients, threshold);

            // Extract the coefficients of the plane of ecuation Ax + By + Cz + D = 0
            double A = coefficients->values[0];
            double B = coefficients->values[1];
            double C = coefficients->values[2];

            // Calculate the normal vector of the plane and normalize it
            normal = Eigen::Vector3d(A, B, C).normalized();

            // Calculate angle between the two vector using the dot product
            double cos_angle = prev_normal.dot(normal);
            double angle = std::acos(cos_angle);
            
            // If the angle between the planes is too big, the plane is not ground
            if (angle > angle_threshold)
            {
                // Store the points in cloud filtered as it is not ground
                *cloud_filtered += *temp_plane;

                // Call the function again until the ground is suitable
                ransac_checking_normal_vectors(temp_filtered, new_temp_filtered, new_temp_plane, coefficients, threshold, cloud_filtered, cloud_plane, 
                    prev_normal, normal, angle_threshold, minimum_ransac_points);
            }

            if (new_temp_filtered->points.empty())
            {
                // Store the ground and not ground points in the specified clouds
                *cloud_filtered += *temp_filtered;
                *cloud_plane += *temp_plane;
            }
            else
            {
                // Store the ground and not ground points in the specified clouds
                *cloud_filtered += *new_temp_filtered;
                *cloud_plane += *new_temp_plane;
            }
        }
    }

    /**
    * @brief Principal function of the file, divide the cloud in the specified numbers of squares as a grid and call the function 
    * ransac_checking_normal_vectors on each square, in order to apply ransac checking verifying that what is filtered is the ground. 
    * @param cloud The raw point cloud that will be filtered.
    * @param cloud_filtered The point cloud that will store the not planar points.
    * @param cloud_plane The point cloud that will store the planar points.
    * @param coefficients The coefficients of the planar ecuation.
    * @param threshold The threshold that will determine if the point belong to the ground.
    * @param Mx The lenght in the x axis.
    * @param My The lenght in the y axis.
    * @param Mz The lenght in the z axis.
    */
    void grid_ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_plane, pcl::ModelCoefficients::Ptr& coefficients, double threshold, double Mx, double My, double Mz,
        int number_sections, double angle_threshold, int minimum_ransac_points)
    {
        // Initialize the variables
        Eigen::Vector3d prev_normal(0, 0, 1);
        Eigen::Vector3d normal(0, 0, 1);

        // Transform the angle threshold from degree to radians
        angle_threshold *= (M_PI/180);

        // Define the measures if the grid
        double x_step = (Mx - 0) / number_sections;
        double y_step = (My - (-My)) / number_sections;

        // Iterate on each square
        for (int i = 0; i < number_sections; ++i)
        {
            for (int j = 0; j < number_sections; ++j)
            {
                // Define the square
                Eigen::Vector4f min_pt(0 + i * x_step, -My + j * y_step, -100.0, 1.0);
                Eigen::Vector4f max_pt(0 + (i + 1) * x_step, -My + (j + 1) * y_step, Mz, 1.0);

                // Crop the input cloud to the square measures
                pcl::PointCloud<pcl::PointXYZI>::Ptr grid_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::CropBox<pcl::PointXYZI> crop_box_filter;
                crop_box_filter.setInputCloud(cloud);
                crop_box_filter.setMin(min_pt);
                crop_box_filter.setMax(max_pt);
                crop_box_filter.filter(*grid_cloud);

                // Create temporal clouds to store the ground and not ground points
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp_filtered(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp_plane(new pcl::PointCloud<pcl::PointXYZI>);
                
                // Apply ransac and check the normal vectors
                ransac_checking_normal_vectors(grid_cloud, temp_filtered, temp_plane, coefficients, threshold, cloud_filtered, cloud_plane, 
                    prev_normal, normal, angle_threshold, minimum_ransac_points);

                // Update the variables for the next iteration
                prev_normal = normal;
            }
        }
    }
} 