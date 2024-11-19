/**
 * @file cropping.h
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to cropp the point cloud received from the lidar.
 * @version 0.1
 * @date 15-11-2024
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Cropping{
    /**
    * @brief Create crop filter function for the point cloud.
    * @param lidar_msg The point cloud that will be cropped.
    */
    void crop_filter_cropbox(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double& Mx, double& My, double& Mz)
    {
        //Configure the cropping filter
        pcl::CropBox<pcl::PointXYZI> crop_box_filter;
        crop_box_filter.setInputCloud(cloud);
        crop_box_filter.setMin(Eigen::Vector4f(0, -My, -100.0, 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(Mx, My, Mz, 1.0));
    
        //Store the cropped cloud
        crop_box_filter.filter(*cloud);
    }   

    /**
    * @brief Create crop filter function for the point cloud.
    * @param lidar_msg The point cloud that will be cropped.
    */
    void crop_filter_condition(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double& Mx, double& My, double& Mz, double& H)
    {
        //Create the condition 
        auto condition = [Mx, My, Mz, H](const pcl::PointXYZI &point)
        {
        return !(point.x < Mx && abs(point.y) < My && point.z < Mz 
        && abs(atan2(point.y, point.x)) < H / 2 && (abs(point.y) > 0.8 || point.x > 2));
        };

        //Filter by the condition
        cloud->erase(std::remove_if(cloud->points.begin(), cloud->points.end(), condition), cloud->points.end());
    }
}
