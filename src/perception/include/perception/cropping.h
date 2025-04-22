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

namespace Cropping
{
    /**
    * @brief Create crop filter function for the point cloud.
    * @param cloud The point cloud that will be cropped.
    * @param Mx The lenght in the x axis.
    * @param My The lenght in the y axis.
    * @param Mz The lenght in the z axis.
    */
    void crop_filter_cropbox(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double& Mx, double& My, double& Mz)
    {
        //Configure the cropping filter
        pcl::CropBox<pcl::PointXYZI> crop_box_filter;
        crop_box_filter.setInputCloud(cloud);
        crop_box_filter.setMin(Eigen::Vector4f(-Mx, -My, -Mz, 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(Mx, My, Mz, 1.0));
    
        //Store the cropped cloud
        crop_box_filter.filter(*cloud);
    }   
}
