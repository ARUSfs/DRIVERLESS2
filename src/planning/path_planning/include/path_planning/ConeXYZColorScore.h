#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


#define UNCOLORED -1
#define BLUE 0
#define YELLOW 1
#define ORANGE 2
#define ORANGE_BIG 3

struct ConeXYZColorScore {
    PCL_ADD_POINT4D;  
    int color;
    float score;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructors
    ConeXYZColorScore() : x(0), y(0), z(0), color(0), score(0) {}
    ConeXYZColorScore(float x, float y, float z, int color, float score) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->color = color;
        this->score = score;
        this->data[3] = 1.0f; 
    }
};


POINT_CLOUD_REGISTER_POINT_STRUCT (ConeXYZColorScore,       
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (int, color, color)
                                   (float, score, score)
)


inline bool operator==(const ConeXYZColorScore& lhs, const ConeXYZColorScore& rhs) {
    return std::abs(lhs.x-rhs.x)<0.1 &&
            std::abs(lhs.y-rhs.y)<0.1 && 
            lhs.color == rhs.color;
}


inline std::string to_string(const ConeXYZColorScore& cone) {
    std::ostringstream oss;
    oss << "ConeXYZColorScore(x: " << cone.x 
        << ", y: " << cone.y 
        << ", z: " << cone.z 
        << ", color: " << cone.color 
        << ", score: " << cone.score << ")";
    return oss.str();
}