/**
* @file PointXYZProbColorScore.h
* @author Rafael Guil Valero (rafaguilvalero@gmail.com)
* @brief Auxiliar file for the Graph Slam node.
* Defines the structure of the custom type PointXYZProbColorScore.
* @version 0.1
* @date 8-6-2025
*/

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#pragma once

struct PointXYZProbColorScore
{
    //definition of the type
    PCL_ADD_POINT4D;  
    double prob_yellow;
    double prob_blue;
    float score;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //Default constructor
    PointXYZProbColorScore()
        : prob_yellow(0), prob_blue(0), score(0.0f) {
        x = y = z = 0.0f;
    }

    //Constructor with parameters
    PointXYZProbColorScore(float x, float y, float z, float prob_yellow, float prob_blue, float score) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->prob_yellow = prob_yellow;
        this->prob_blue = prob_blue;
        this->score = score;
        this->data[3] = 1.0f; 
    }
};

//Registering custom type in PCL
POINT_CLOUD_REGISTER_POINT_STRUCT (
    PointXYZProbColorScore,       
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (double, prob_yellow, prob_yellow)
    (double, prob_blue, prob_blue)
    (float, score, score)
)