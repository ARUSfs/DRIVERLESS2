/**
* @file PointXYZIRingTime.h
* @author Álvaro Galisteo Bermúdez (galisbermo03@gmail.com)
* @brief Auxiliar file for the Perception node.
* Defines the structure of the custom type PointXYZIRingTime.
* @version 0.1
* @date 15-3-2025
*/

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#pragma once

struct PointXYZIRingTime
{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    float timestamp;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointXYZIRingTime() : x(0), y(0), z(0), intensity(0), ring(0), timestamp(0)
    {
    }
    PointXYZIRingTime(float x, float y, float z, float intensity, uint16_t ring, float timestamp)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->intensity = intensity;
        this->ring = ring;
        this->timestamp = timestamp;
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRingTime,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (double, timestamp, timestamp))