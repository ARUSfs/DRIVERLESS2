/**
 * @file accumulation.h
 * @author Rafael Guil Valero (rafaguilvalero@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to accumulate the PCL received from the lidar.
 * @version 0.1
 * @date 17-05-2025
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <deque>
#include <iostream>
#include "PointXYZIRingTime.h"
#include <pcl/filters/voxel_grid.h> 
#include "perception_acc/clustering.h"
#include <chrono>
#include <boost/circular_buffer.hpp>
#include "PointXYZIRingTime.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


namespace Accumulation
{
    static boost::circular_buffer<pcl::PointCloud<PointXYZIRingTime>::Ptr> cloud_buffer;
    static bool buffer_cloud_initialized = false;
    static double pcl_inclination_ = 0.0; // Inclination of the PCL

    /**
     * @brief Computes the rigid transformation matrix based on the given parameters.
     * @param x Translation along X-axis
     * @param y Translation along Y-axis
     * @param yaw Rotation around Z-axis (yaw)
     * @param kDistanceLidarToCoG Distance from LIDAR to CoG
     * @return Rigid transformation matrix
     */
    Eigen::Isometry3d computeRigidTransformation(double x, double y, double yaw, double kDistanceLidarToCoG) {
        // Transformation from LIDAR to CoG
        Eigen::Isometry3d transform_lidar_to_CoG = Eigen::Isometry3d::Identity();
        transform_lidar_to_CoG.translation() << -kDistanceLidarToCoG, 0.0, 0.0;

        // Motion transformation matrix (rotation + translation)
        Eigen::Isometry3d transform_motion = Eigen::Isometry3d::Identity();
        transform_motion.linear() << cos(yaw), -sin(yaw), 0,
                                      sin(yaw),  cos(yaw), 0,
                                      0,         0,        1;
        transform_motion.translation() << x, y, 0.0;

        // Transformation from CoG back to LIDAR
        Eigen::Isometry3d transform_CoG_to_lidar = Eigen::Isometry3d::Identity();
        transform_CoG_to_lidar.translation() << kDistanceLidarToCoG, 0.0, 0.0;

        // Compute final transformation matrix
        Eigen::Isometry3d final_transform = transform_lidar_to_CoG * transform_motion * transform_CoG_to_lidar;

        return final_transform;  // Return the transformation matrix
    }

    void estimate_inclination(pcl::PointCloud<PointXYZIRingTime>::Ptr& cloud)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::SACSegmentation<PointXYZIRingTime> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10);
        seg.setDistanceThreshold(0.2);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            return; 
        }

        Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Vector3d up(0.0, 0.0, 1.0);
        double angle = std::acos(normal.normalized().dot(up));
        pcl_inclination_ = angle;

        std::cout << "Inclination angle: " << angle * 180.0 / M_PI << " degrees" << std::endl;
    }

    inline void disinclinate_cloud(pcl::PointCloud<PointXYZIRingTime>::Ptr& cloud)
    {
        Eigen::AngleAxisd rotation(-pcl_inclination_, Eigen::Vector3d::UnitY());
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3,3>(0,0) = rotation.toRotationMatrix();
        pcl::transformPointCloud(*cloud, *cloud, transform);
    }

    void accumulate(pcl::PointCloud<PointXYZIRingTime>::Ptr cloud, 
                    pcl::PointCloud<PointXYZIRingTime>::Ptr& accumulated_cloud,
                    double x, double y, double yaw, double kDistanceLidarToCoG,
                    double kBufferSize)
    {
        // Clean the buffer the first time
        if (!buffer_cloud_initialized) {
            cloud_buffer.clear();
            cloud_buffer.set_capacity(kBufferSize);
            buffer_cloud_initialized = true;
            cloud_buffer.push_back(cloud);
            return; // Removed invalid return value.
        }

        if (!cloud || cloud->empty()) {
            return; // Removed invalid return value.
        }

        auto final_transform = computeRigidTransformation(x, y, yaw, kDistanceLidarToCoG);

        cloud_buffer.push_front(cloud);

        for (int i = 0; i < cloud_buffer.size(); i++)
        {
            *accumulated_cloud += *cloud_buffer[i];
        }

        // estimate_inclination(cloud);
        // disinclinate_cloud(accumulated_cloud);

        // Revert to original position before returning
        pcl::transformPointCloud(*accumulated_cloud, *accumulated_cloud, final_transform.inverse().matrix());        
    }

}