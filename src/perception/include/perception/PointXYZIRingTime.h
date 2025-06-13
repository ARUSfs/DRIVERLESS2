/**
* @file PointXYZColorScore.h
* @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
* @brief Defines the structure of the custom type PointXYZIRingTime.
*/

#include <pcl/io/io.h>
#include <pcl/point_types.h>

#pragma once

struct PointXYZIRingTime
{
    // Definition of the type
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EIGEN_ALIGN16

    // Default constructor
    PointXYZIRingTime()
        : intensity(0.0f), ring(0), timestamp(0.0) {
        x = y = z = 0.0f;
    }

    // Constructor with parameters
    PointXYZIRingTime(float x, float y, float z, float intensity, std::uint16_t ring, double timestamp) 
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->intensity = intensity;
        this->ring = ring;
        this->timestamp = timestamp;
        this->data[3] = 1.0f; 
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRingTime, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)