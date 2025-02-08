/**
 * @file utils.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @date 17-01-2025
 * @brief Header file containing common functions to use in several files.
 * 
 * @date 17-01-2025
 */

#pragma once

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

/**
 * @brief Calculate the centroid of a triangle given its index in the triangulation.
 * Centroid is calculated as the arithmetic mean of its three vertices in each coordinate. 
 * The centroid always lies inside the triangle's convex hull.
 * @param triangle_ind int index of the triangle in the triangulation.
 * @param triangles TriangleVec list of triangles in the triangulation.
 * @param vertices CDT::Triangulation<double>::V2dVec list of vertices in the triangulation.
 * @return CDT::V2d<double> Centroid of the triangle as a CDT point.
 */
CDT::V2d<double> compute_centroid(int triangle_ind, CDT::TriangleVec triangles, CDT::Triangulation<double>::V2dVec vertices){
    CDT::V2d<double> a, b, c;
    CDT::Triangle triangle = triangles[triangle_ind];
    CDT::VerticesArr3 vertices_index = triangle.vertices;
    CDT::VertInd a_ind = vertices_index[0];
    CDT::VertInd b_ind = vertices_index[1];
    CDT::VertInd c_ind = vertices_index[2];
    a = vertices[a_ind];
    b = vertices[b_ind];
    c = vertices[c_ind];
    CDT::V2d<double> centroid = CDT::V2d<double>::make((a.x+b.x+c.x)/3, (a.y+b.y+c.y)/3);
    return centroid;
}

template <typename T>
bool in(const T& value, const std::vector<T>& container) {
    return std::find(container.begin(), container.end(), value) != container.end();
}

bool in(unsigned int& value, CDT::VerticesArr3& container) {
    return std::find(container.begin(), container.end(), value) != container.end();
}

bool in(unsigned int& value, std::vector<int>& container) {
    return std::find(container.begin(), container.end(), value) != container.end();
}