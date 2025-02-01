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

/**
 * @brief Calculate the similarity between two lists of points.
 * Similarity is defined as the number of points in list1 that are in list2 (with user-defined 
 * tolerance). It is assumed that points do not repeat in the lists.
 * @param list1 vector of points
 * @param list2 vector of points
 * @param tolerance 0.1 by default, maximum distance between points to consider them the same
 * @return int number of points in both lists
 */
int compare_lists(std::vector<CDT::V2d<double>> list1, std::vector<CDT::V2d<double>> list2, double tolerance=0.1){
    int count = 0;
    for (int i = 0; i<list1.size(); i++){
        for (int j = 0; j<list2.size(); j++){
            if (distance(list1[i], list2[j])<tolerance){
                count++;
                break;
            }
        }
    }
    return count;
}