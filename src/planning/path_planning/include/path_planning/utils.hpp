/**
 * @file utils.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @date 17-01-2025
 * @brief Header file containing common functions to use in several files.
 * 
 * @date 17-01-2025
 */
#include <cmath>
#include <iostream>

#include "ConeXYZColorScore.h"
#include "CDT.h"
#include <Triangulation.h>
#include <CDTUtils.h>

/**
 * @brief Calculate the euclidean distance between two cones.
 * @param a ConeXYZColorScore first cone
 * @param b ConeXYZColorScore second cone
 * @return double distance between the cones
 */
double distance(ConeXYZColorScore a, ConeXYZColorScore b){
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y,2) + pow(a.z-b.z,2));
}