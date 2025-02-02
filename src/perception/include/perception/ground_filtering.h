/**
 * @file ground_filtering.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to remove the ground from the original point cloud.
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
    * @brief Implements a specific structure, that consists of a vector of points, used in the novel ground filter.
    */
    struct Segment 
    {
        std::vector<pcl::PointXYZI> points;
    }; 

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
    * @brief Auxiliar function for grid ground filter, apply ransac in the square specified and check if the normal vector is close to be paralel to the previus
    * normal vector (that is ground), if not, repeat the process after taking out the false ground.
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
            GroundFiltering::ransac_ground_filter(grid_cloud, temp_filtered, temp_plane, coefficients, threshold);

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
    * @brief Divide the cloud in the specified numbers of squares as a grid and apply ransac segmentation on each square.
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
    
    /**
    * @brief Auxiliar function for novel ground filter, divide the cloud in the in circular rings and each ring in segments.
    * @param cloud The raw point cloud that will be divided.
    * @param num_rings The number of rings.
    * @param num_sectors The number of sectors.
    */
    void divide_into_segments(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<std::vector<Segment>>& segments, int num_rings, int num_sectors, 
        double max_radius)
    {  
        // Resize the empty segments
        segments.resize(num_rings, std::vector<Segment>(num_sectors));

        // Iterate on each point
        for (const auto& point : cloud->points)
        {
            // Transform the cartesian coordinates to polar coordinates.
            double r = std::sqrt(point.x * point.x + point.y * point.y);
            double theta = std::atan2(point.y, point.x);
            if (theta < 0) theta += 2 * M_PI;

            // Assign the correct indices of the ring and sector to the point
            int ring_idx = std::min(static_cast<int>(num_rings * r / max_radius), num_rings - 1);
            int sector_idx = std::min(static_cast<int>(num_sectors * theta / (2 * M_PI)), num_sectors - 1);

            // Store the point in the sector
            segments[ring_idx][sector_idx].points.push_back(point);
        }
    }

    /**
    * @brief Auxiliar function for novel ground filter, find the lowest point of the segment specifies.
    * @param segment The segment.
    */
    pcl::PointXYZI find_lowest_point(const Segment& segment)
    {
        // Define what will be the lowest point
        pcl::PointXYZI lowest_point;

        // Initialize the maximun z to compare
        double min_z = std::numeric_limits<double>::max();

        // Iterate on each point comparing its z coordinate
        for (const auto& point : segment.points) 
        {
            if (point.z < min_z) 
            {
                // If a point with less z coordinate is found, it is the new lowest point
                min_z = point.z;
                lowest_point = point;
            }
        }

        // Return the lowest point
        return lowest_point;
    }

    /**
    * @brief Auxiliar function for novel ground filter, calculates the proyection of a point in a plane.
    * @param ppoint The point.
    * @param plane The plane.
    */
    pcl::PointXYZI project_point_to_plane(const pcl::PointXYZI& point, const Eigen::Vector4f& plane) 
    {
        // Calculate the normal vector of the plane and the distance from the point to the plane
        Eigen::Vector3f normal(plane[0], plane[1], plane[2]);

        // Calculate the distance from the point to the plane using the formula (Ax + By + Cz + D) / (A² + B² * c²)^(1/2)
        double distance = (plane[0] * point.x + plane[1] * point.y + plane[2] * point.z + plane[3]) /
            normal.norm();

        // Calculate the proyection of the point in the plane using the formula pxi = xi - distance * coef(xi)
        pcl::PointXYZI projection;
        projection.x = point.x - distance * plane[0];
        projection.y = point.y - distance * plane[1];
        projection.z = point.z - distance * plane[2];

        // Return the proyection
        return projection;
    }

    /**
    * @brief Auxiliar function for novel ground filter, calculates the coeficients of a planar ecuation as ax + by + cz + d = 0, from three points.
    * @param p1 The first point.
    * @param p2 The second point.
    * @param p3 The third point.
    */
    Eigen::Vector4f fit_plane(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2, const pcl::PointXYZI& p3) 
    {
        // Calculates vectors from p1 to p2 and from p1 to p3
        Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        Eigen::Vector3f v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

        // Calculates the coefficients a, b and c using the cross product
        Eigen::Vector3f normal = v1.cross(v2).normalized();

        // Determine the value of d
        double d = -normal.dot(Eigen::Vector3f(p1.x, p1.y, p1.z));

        // Return the coefficients
        return Eigen::Vector4f(normal[0], normal[1], normal[2], d);
    }

    /**
    * @brief Auxiliar function for novel ground filter, adjust the a suitable plane to each segment
    * @param segments The segments.
    * @param planes The output planes.
    * @param angle_threshold The threshold to compare the normal vector of two consecutives planes.
    */
    void fit_planes(const std::vector<std::vector<Segment>>& segments, std::vector<Eigen::Vector4f>& planes, double angle_threshold)
    {
        // iterate on each segment
        for (size_t ring_idx = 0; ring_idx < segments.size(); ++ring_idx) 
        {
            for (size_t sector_idx = 0; sector_idx < segments[ring_idx].size(); ++sector_idx) 
            {
                // Filter the empty segments
                if (segments[ring_idx][sector_idx].points.empty()) continue;

                // Find the lowest point of the segment
                pcl::PointXYZI p1 = find_lowest_point(segments[ring_idx][sector_idx]);

                if (ring_idx > 0 && sector_idx > 0) 
                {
                    pcl::PointXYZI p2 = find_lowest_point(segments[ring_idx - 1][sector_idx]);
                    pcl::PointXYZI p3 = find_lowest_point(segments[ring_idx - 1][(sector_idx - 1) % segments[ring_idx].size()]);

                    Eigen::Vector4f plane = fit_plane(p1, p2, p3);

                    if (!planes.empty()) 
                    {
                        Eigen::Vector3f normal_current(plane[0], plane[1], plane[2]);
                        Eigen::Vector3f normal_previous(planes.back()[0], planes.back()[1], planes.back()[2]);

                        // Calculate angle between the two vector using the dot product
                        double cos_angle = normal_current.dot(normal_previous);
                        double angle = std::acos(cos_angle);

                        // If the plane is not ground discard it
                        if (angle > angle_threshold) continue;
                    }

                    planes.push_back(plane);
                }
            }
        } 
    }

    /**
    * @brief Auxiliar function for novel ground filter, adjust the a suitable plane to each segment
    * @param segments The segments.
    * @param planes The output planes.
    * @param angle_threshold The threshold to compare the normal vector of two consecutives planes.
    * @param num_rings The number of rings.
    * @param num_sectors The number of sectors.
    */
    void fit_planes2(const std::vector<std::vector<Segment>>& segments, std::vector<Eigen::Vector4f>& planes, double angle_threshold, double num_rings, 
    double num_sectors)
    {
        // Initialize the neccesary variables
        std::vector<std::vector<pcl::PointXYZI>> centers;
        Eigen::Vector3f prev_normal(0, 0, 1); 

        // Resize the centers
        centers.resize(num_rings, std::vector<pcl::PointXYZI>(num_sectors));

        // Iterate on each segment
        for (size_t ring_idx = 0; ring_idx < segments.size(); ++ring_idx) 
        {
            for (size_t sector_idx = 0; sector_idx < segments[ring_idx].size(); ++sector_idx) 
            {
                // Rename the segment for easier coding
                const auto& segment = segments[ring_idx][sector_idx];
                
                // Validation in case of the segment is empty
                if (segment.points.empty()) continue;

                // Define the points that will compose the plane
                pcl::PointXYZI p1;
                pcl::PointXYZI p2;
                pcl::PointXYZI p3;

                // Define the first point
                if (ring_idx == 0)
                {
                    // If we are working on the first ring, the first point is the center (right below the lidar) with the lowest z coordinate in the segment
                    pcl::PointXYZI temp_point = find_lowest_point(segment);
                    p1.x = 0.0;
                    p1.y = 0.0;
                    p1.z = temp_point.z;
                }
                else
                {
                    // If we are not working on the first ring, the first point is the center of the plane of the same segment in the previus ring
                    p1 = centers[ring_idx - 1][sector_idx];
                }

                // Define the second and thrid point
                if (sector_idx == 0)
                {
                    // If we are working in the first sector, the second are third point are the lowest of the segment and the lowest of the neighbor, respectively
                    p2 = find_lowest_point(segment);
                    p3 = find_lowest_point(segments[ring_idx][sector_idx + 1]);
                }
                else if (sector_idx == segments[ring_idx].size() - 1)
                {
                    // If we are working in the last sector, the second are third point are the lowest of the neighbor and the lowest of the segment, respectively
                    p2 = find_lowest_point(segment);
                    p3 = find_lowest_point(segments[ring_idx][sector_idx - 1]);
                }
                else
                {
                    // If we are working in the first sector, the second are third point are the lowest of the neighbors
                    p2 = find_lowest_point(segments[ring_idx][sector_idx + 1]);
                    p3 = find_lowest_point(segments[ring_idx][sector_idx - 1]);
                }

                // Calculate the plane with the three points
                Eigen::Vector4f plane = fit_plane(p1, p2, p3);

                // Calculate the vectors v1 (from p2 to p1) and v2 (from p3 to p1)
                Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
                Eigen::Vector3f v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

                // Calculate the normal from v1 and v2
                Eigen::Vector3f normal = v1.cross(v2).normalized();

                // Compare the normal vector of this segment to the one of the previus segment
                if (!planes.empty()) 
                {
                    // Calculate angle between the two vector using the dot product
                    double cos_angle = prev_normal.dot(normal);
                    double angle = std::acos(cos_angle);

                    // If the plane is not ground discard it
                    if (angle > angle_threshold) continue;
                }

                // Update the variables for the next iteration
                prev_normal = normal;

                pcl::PointXYZI center;
                center.x = (p1.x + p2.x + p3.x) / 3.0f;
                center.y = (p1.y + p2.y + p3.y) / 3.0f;
                center.z = (p1.z + p2.z + p3.z) / 3.0f;
                centers[ring_idx][sector_idx] = project_point_to_plane(center, plane);

                // Store the suitable plane
                planes.push_back(plane);
            }
        } 
    }

    /**
    * @brief Auxiliar function for novel ground filter, determine if a point belogns to the ground or not.
    * @param poins The points.
    * @param planes The adjusted planes.
    * @param threshold The threshold that will determine if the point belong to the ground.
    */
    bool is_ground(const pcl::PointXYZI& point, const std::vector<Eigen::Vector4f>& planes, double threshold) 
    {
        // Iterate on each plane
        for (const auto& plane : planes) 
        {
            // Calculate the distance from the point to the plane using the formula (Ax + By + Cz + D) / (A² + B² * c²)^(1/2)
            double distance = std::abs(plane[0] * point.x + plane[1] * point.y + plane[2] * point.z + plane[3]) /
                std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);

            if (distance < threshold) return true;
        }

        return false;
    }            
    
    /**
    * @brief Personalized filter to separate ground and not grounf points, as a result, it is used to filter the ground.
    * @param cloud The raw point cloud that will be filtered.
    * @param cloud_filtered The point cloud that will store the not planar points.
    * @param cloud_plane The point cloud that will store the planar points.
    * @param threshold The threshold that will determine if the point belong to the ground.
    */
    void novel_ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_plane, double threshold, int num_rings, int num_sectors, double max_radius, double angle_threshold)
    {
        // Transform the angle threshold from degree to radians
        angle_threshold *= (M_PI/180);

        // Divide the point cloud in segments
        std::vector<std::vector<Segment>> segments;
        divide_into_segments(cloud, segments, num_rings, num_sectors, max_radius);

        // Find the suitable plane for each segment
        std::vector<Eigen::Vector4f> planes;
        fit_planes2(segments, planes, angle_threshold, num_rings, num_sectors);

    
        // Iterate on each point to decide if is part of the ground or not
        for (const auto& point : cloud->points) 
        {
            if (is_ground(point, planes, threshold)) 
            {
                cloud_plane->points.push_back(point);
            } 
            else 
            {
                cloud_filtered->points.push_back(point);
            }
        }

        // Resize cloud filtered
        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = true;

        // Resize the cloud plane
        cloud_plane->width = cloud_plane->points.size();
        cloud_plane->height = 1;
        cloud_plane->is_dense = true;
    }
} 