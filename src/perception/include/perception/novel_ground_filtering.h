/**
 * @file novel_ground_filtering.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains a specific procedure used to remove the ground from the original point cloud. It is a method that consists of dividing
 * the point cloud in concentric rings and each ring in sectors. The function assigns to each sector three sepecific points depending on
 * their ring and sector and it is estimated and removed the ground using these three points.
 * @version 0.1
 * @date 3-2-2025
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

namespace novel_ground_filtering
{
    /**
    * @brief Implements a specific structure, that consists of a vector of points, used in the novel ground filter.
    */
    struct Segment 
    {
        std::vector<pcl::PointXYZI> points;
    };

    /**
    * @brief Auxiliar function for  the funciton novel_ground_filter, divide the cloud in the circular rings and each ring in segments.
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
    * @brief Auxiliar function for  the funciton novel_ground_filter, find the lowest point of the segment specifies 
    * by comparing the z coordinate of each point.
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
    * @brief Auxiliar function for  the funciton novel_ground_filter, calculates the proyection of a point in a plane 
    * using a mathematical formula.
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
    * @brief Auxiliar function for  the funciton novel_ground_filter, calculates the coeficients of a planar ecuation as ax + by + cz + d = 0, from three points.
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
    * @brief Auxiliar function for  the funciton novel_ground_filter, adjust a suitable plane to each segment following this criterion: 
    * The first point depends on the ring:
    * -> first ring, select the origin point with the z coordinate modified.
    * -> rest of the rings, select the projection of the center point of the same sector in the previus ring in the ground plane of that segment.
    * The second and thrid point depend on the sector:
    * -> first or last sector, selects the lowest points on the sector and on the neighbor sector.
    * -> rest of the sectors, selects the lowest points on the neighbors sectors.
    * @param segments The segments.
    * @param planes The output planes.
    * @param angle_threshold The threshold to compare the normal vector of two consecutives planes.
    * @param num_rings The number of rings.
    * @param num_sectors The number of sectors.
    */
    void fit_planes(const std::vector<std::vector<Segment>>& segments, std::vector<Eigen::Vector4f>& planes, double angle_threshold, 
    double num_rings, double num_sectors)
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
    * @brief Auxiliar function for  the funciton novel_ground_filter, determine if a point belogns to the ground or not, 
    * by measuring the distane between the plane and the point and comparing it to a certain threshold.
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
    * @brief Principal function of the file, personalized filter to separate ground and not grounf points, as a result, it is used to filter the ground.
    * It divides the point cloud in ring and sectors, and fit a ground plane to each sector, by calculating the plane defined from three selected points
    * depending of the localization of the sector.
    * @param cloud The raw point cloud that will be filtered.
    * @param cloud_filtered The point cloud that will store the not planar points.
    * @param cloud_plane The point cloud that will store the planar points.
    * @param threshold The threshold that will determine if the point belong to the ground.
    */
    void novel_ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_plane, double threshold, int num_rings, int num_sectors, double max_radius, 
        double angle_threshold)
    {
        // Transform the angle threshold from degree to radians
        angle_threshold *= (M_PI/180);

        // Divide the point cloud in segments
        std::vector<std::vector<Segment>> segments;
        divide_into_segments(cloud, segments, num_rings, num_sectors, max_radius);

        // Find the suitable plane for each segment
        std::vector<Eigen::Vector4f> planes;
        fit_planes(segments, planes, angle_threshold, num_rings, num_sectors);

    
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