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
 * @brief Get the index of a given vertex in the triangulation. Returns -1 if not found.
 */
int get_vertex_index(CDT::V2d<double> vertex, CDT::Triangulation<double> triangulation, double tolerance = 0.0){
    int o_ind;
    CDT::Triangulation<double>::V2dVec vertices = triangulation.vertices;
    if (tolerance == 0.0){
        for (int i = 0; i<vertices.size(); i++){
            if (vertices[i] == vertex){
                o_ind = i;
                return o_ind;
            }
        }
    } else {
        for (int i = 0; i<vertices.size(); i++){
            if (distance(vertices[i], vertex) < tolerance){
                o_ind = i;
                return o_ind;
            }
        }
    }
    return -1;
}

/**
 * @brief Get the triangles adjacent to a vertex from its index.
 */
std::vector<int> get_triangles_from_vert(int vert_index, CDT::Triangulation<double> triangulation){
    std::vector<int> o_triangles;
    CDT::TriangleVec triangles = triangulation.triangles;
    for (int i = 0; i<triangles.size(); i++){
        if (triangles[i].containsVertex(vert_index)){
            o_triangles.push_back(i);
        }
    }
    return o_triangles;
}

/**
 * @brief Calculate the euclidean distance between two cones.
 */
double distance(ConeXYZColorScore a, ConeXYZColorScore b){
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y,2));
}

/**
 * @brief Calculate the similarity between two lists of points.
 * Similarity is defined as the number of points in list1 that are in list2 (with user-defined 
 * tolerance). It is assumed that points do not repeat in the lists.
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
 * @brief Calculate the similarity between two lists of points. Overloaded for ConeXYZColorScore.
 * Similarity is defined as the number of points in list1 that are in list2 (with user-defined 
 * tolerance). It is assumed that points do not repeat in the lists.
 */
int compare_lists(std::vector<ConeXYZColorScore> list1, std::vector<ConeXYZColorScore> list2, double tolerance=0.1){
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

/**
 * @brief Check if a value is contained in a vector. Overloaded function.
 */
template <typename T>
bool in(const T& value, const std::vector<T>& container) {
    return std::find(container.begin(), container.end(), value) != container.end();
}

/**
 * @brief Check if a value is contained in a vector. Overloaded function.
 */
bool in(unsigned int& value, CDT::VerticesArr3& container) {
    return std::find(container.begin(), container.end(), value) != container.end();
}

/**
 * @brief Check if a value is contained in a vector. Overloaded function.
 */
bool in(unsigned int& value, std::vector<int>& container) {
    return std::find(container.begin(), container.end(), value) != container.end();
}

std::vector<double> laplacian_smoothing(std::vector<double> input, double coef = 0.5) {
    std::vector<double> output;
    if (input.size() < 3) {
        return input; // Not enough points to smooth
    }
    output.push_back(input[0]); // First element remains unchanged
    for (int i = 1; i < input.size() - 1; i++) {
        double smoothed_value = 0.5*(1-coef)*input[i-1] + coef*input[i] + 0.5*(1-coef)*input[i+1];
        output.push_back(smoothed_value);
    }
    output.push_back(input.back()); // Last element remains unchanged
    return output;
}