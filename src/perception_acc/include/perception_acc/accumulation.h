/**
 * @file accumulation.h
 * @author Álvaro Galisteo Bermúdez (galisbermo03@gmail.com)
 * @brief Auxiliar file for the Perception node.
 * Contains auxiliar functions used in the algorithm to accumulate the point cloud received from the lidar.
 * @version 0.1
 * @date 12-02-2025
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
        transform_CoG_to_lidar.translation() << kDistanceLidarToCoG, 0.0, 2.0;

        // Compute final transformation matrix
        Eigen::Isometry3d final_transform = transform_lidar_to_CoG * transform_motion * transform_CoG_to_lidar;

        return final_transform;  // Return the transformation matrix
    }

    void disinclinate_cloud(pcl::PointCloud<PointXYZIRingTime>::Ptr& cloud)
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

        Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Vector3f up(0.0f, 0.0f, 1.0f);
        float angle = acos(normal.normalized().dot(up));
        Eigen::Vector3f axis = normal.normalized().cross(up).normalized();
        Eigen::AngleAxisf rotation(angle, axis);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = rotation.toRotationMatrix();

        pcl::transformPointCloud(*cloud, *cloud, transform);

        // Find the lowest z in the cloud
        float min_z = std::numeric_limits<float>::max();
        for (auto &pt : cloud->points) {
            if (pt.z < min_z) {
                min_z = pt.z;
            }
        }

        // Translate all points so lowest z is 0
        Eigen::Matrix4f shiftMat = Eigen::Matrix4f::Identity();
        shiftMat(2,3) = -min_z;
        pcl::transformPointCloud(*cloud, *cloud, shiftMat);
    }

    inline void disinclinate_cloud2(pcl::PointCloud<PointXYZIRingTime>::Ptr& cloud, double pcl_inclination)
    {
        float angle_rad = pcl_inclination * M_PI / 180.0f; 
        Eigen::AngleAxisf rotation(-angle_rad, Eigen::Vector3f::UnitY());
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = rotation.toRotationMatrix();
        pcl::transformPointCloud(*cloud, *cloud, transform);
    }

    void accumulate(pcl::PointCloud<PointXYZIRingTime>::Ptr cloud, 
                    pcl::PointCloud<PointXYZIRingTime>::Ptr& accumulated_cloud,
                    double x, double y, double yaw, double kDistanceLidarToCoG,
                    double kBufferSize, double pcl_inclination)
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

        disinclinate_cloud2(accumulated_cloud, pcl_inclination);

        // Revert to original position before returning
        pcl::transformPointCloud(*accumulated_cloud, *accumulated_cloud, final_transform.inverse().matrix());        
    }

}