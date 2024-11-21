/**
* @file PointXYZColorScore.h
* @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
* @brief Auxiliar file for the Perception node.
* Defines the structure of the custom type PointXYZColorScore.
* @version 0.1
* @date 19-11-2024
*/

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

struct PointXYZColorScore {
    //definition of the type
    PCL_ADD_POINT4D;  
    int color;
    float score;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //Default constructor
    PointXYZColorScore()
        : color(0), score(0.0f) {
        x = y = z = 0.0f;
    }

    //Constructor with parameters
    PointXYZColorScore(float x, float y, float z, int color, float score) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->color = color;
        this->score = score;
        this->data[3] = 1.0f; 
    }
};

//Registering custom type in PCL
POINT_CLOUD_REGISTER_POINT_STRUCT (
    PointXYZColorScore,       
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (int, color, color)
    (float, score, score)
)