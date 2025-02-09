/**
 * @file simplex_tree.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @date 01-11-2024
 * @brief Creates a tree structure to store information about neighbor triangles.
 * It also contain helper functions such as a search function.
 * 
 */
#include <path_planning/simplex_node.hpp>
#include <iostream>
#include <CDT.h>
#include <Triangulation.h>
#include <CDTUtils.h>
#include "utils.hpp"


/**
 * @brief Class for the simlpex tree structure.
 * Nodes are indices of the triangles in the triangulation. Two nodes are connected if they share an edge.
 */
class SimplexTree {
    public:
    /**
     * @brief Right and left children of the node. Recursive structure for tree construction.
     */
    SimplexNode root_;

    /**
     * @brief pcl::PointCloud object containing the cones in the map.
     */
    pcl::PointCloud<ConeXYZColorScore> cones_cloud_;

    /**
     * @brief Array of arrays containing all the posible routes through the tree.
     */
    std::vector<std::vector<int>> index_routes_;

    std::vector<std::vector<ConeXYZColorScore>> midpoint_routes_;

    /**
     * @brief Construct a new generic tree object.
     * @param key int index of the triangle.
     */
    SimplexTree(CDT::TriangleVec triangle_list, int origin, int orig_vertex, 
                                    pcl::PointCloud<ConeXYZColorScore> cones_cloud);

    /**
     * @brief Recursive function to create the tree structure.
     * From a triangle list and an index from which to start, it creates a tree structure containing all 
     * the routes through the triangles that share an edge.
     * Left is the default path, right is created when there are two valid neighbors.
     * @param triangle_list triangulation.triangles object containing all the triangles.
     * @param index triangle index from which to start
     * @param visited array containing the visited triangle in the current route.
     * @param passed_vertices array containing the vertices that have been passed 
     * so any triangle containing them is not visited.
     * @param mid_route array containing the midpoints of the edges in the route.
     * @return SimplexNode* pointer to the created tree.
     */
    SimplexNode* create_tree_aux(CDT::TriangleVec triangle_list, int index, 
                                 std::vector<int> visited, std::vector<int> passed_vertices,
                                 std::vector<ConeXYZColorScore> mid_route);
};

SimplexTree::SimplexTree(CDT::TriangleVec triangle_list, int origin_ind, int orig_vertex,
                            pcl::PointCloud<ConeXYZColorScore> cones_cloud) {
    cones_cloud_ = cones_cloud;

    CDT::Triangle origin = triangle_list[origin_ind]; // Get triangle from index
    CDT::NeighborsArr3 neighbors = origin.neighbors;  // Get neighbors of the triangle

    std::vector<int> visited = {origin_ind}; // Initialize visited array with the origin index
    std::vector<int> passed_vertices = {orig_vertex}; // Initialize visited array with the origin index

    root_.index = origin_ind;                 // Set the root index to the origin index

    std::vector<ConeXYZColorScore> mid_route = {cones_cloud_.points[orig_vertex]}; // Start at the origin

    // Filter neighbors to find which of them are valid
    for (int i = 0; i<3; i++){
        if((neighbors[i]<=triangle_list.size()) && 
                triangle_list[neighbors[i]].vertices[0] != orig_vertex &&
                triangle_list[neighbors[i]].vertices[1] != orig_vertex &&
                triangle_list[neighbors[i]].vertices[2] != orig_vertex){

            visited.push_back(neighbors[i]); // Add the valid neighbors to the visited array
            root_.left = SimplexTree::create_tree_aux(triangle_list, neighbors[i], visited, passed_vertices,
                                                      mid_route);
            break;
        }
    }

} 


SimplexNode* SimplexTree::create_tree_aux(CDT::TriangleVec triangle_list, int index,
                                          std::vector<int> visited, std::vector<int> passed_vertices,
                                          std::vector<ConeXYZColorScore> mid_route) {

    SimplexNode* node = new SimplexNode(index);
    CDT::Triangle triangle = triangle_list[index];     // Get the triangle from the index
    CDT::NeighborsArr3 neighbors = triangle.neighbors; // Get the neighbors of the triangle
    std::vector<ConeXYZColorScore> neighbor_edge = {};
    std::vector<int> valid_neighbors = {};
    for (int i = 0; i<3; i++){
        if (neighbors[i]>triangle_list.size()) continue; 
        
        CDT::Triangle n_triangle = triangle_list[neighbors[i]];
        std::vector<ConeXYZColorScore> next_edge = {};

        for (auto v: n_triangle.vertices){
            if (!in(v, triangle.vertices)){
                if (in(v, passed_vertices)){ // Check the neighbor doesn't contain any passed vertex
                    next_edge.clear();
                    break;
                }
            } else {
                next_edge.push_back(cones_cloud_.points[v]);
            }
        }
        if (next_edge.size() == 2 &&
            (next_edge[0].color == UNCOLORED ||  
             next_edge[1].color == UNCOLORED || 
             next_edge[0].color != next_edge[1].color) && // Check the edge is not colored or is colored with different colors
             distance(next_edge[0], next_edge[1]) > 2.0){ // Check the edge is longer than 2.0 meters
            valid_neighbors.push_back(neighbors[i]);
            neighbor_edge.push_back(ConeXYZColorScore((next_edge[0].x+next_edge[1].x)/2,
                                                      (next_edge[0].y+next_edge[1].y)/2, 0, UNCOLORED, 1));
        }
        
    }
    

    // Only change is that the node is returned when there is only one valid neighbor
    if (valid_neighbors.size() == 1){
        // In case only one neighbor is valid, create the left child and return the node
        visited.push_back(valid_neighbors[0]);
        mid_route.push_back(neighbor_edge[0]);

        CDT::Triangle next_triangle = triangle_list[valid_neighbors[0]];

        for (auto v: triangle.vertices){
            if (!in(v, next_triangle.vertices)){
                passed_vertices.push_back(v);
            }
        }

        node->left = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[0], visited, passed_vertices, mid_route);
        return node;
    }
    else if (valid_neighbors.size() == 2){
        // In case two neighbors are valid, create both children and return the node
        std::vector<int> visited_left = visited;
        std::vector<int> visited_right = visited;
        visited_left.push_back(valid_neighbors[0]);
        visited_right.push_back(valid_neighbors[1]);

        std::vector<ConeXYZColorScore> mid_route_left = mid_route;
        std::vector<ConeXYZColorScore> mid_route_right = mid_route;
        mid_route_left.push_back(neighbor_edge[0]);
        mid_route_right.push_back(neighbor_edge[1]);


        std::vector<int> passed_vertices_left = passed_vertices;
        CDT::Triangle left_triangle = triangle_list[valid_neighbors[0]];
        for (auto v: triangle.vertices){
            if (!in(v, left_triangle.vertices)){
                passed_vertices_left.push_back(v);
            }
        }

        std::vector<int> passed_vertices_right = passed_vertices;
        CDT::Triangle right_triangle = triangle_list[valid_neighbors[1]];
        for (auto v: triangle.vertices){
            if (!in(v, right_triangle.vertices)){
                passed_vertices_right.push_back(v);
            }
        }

        node->left = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[0], visited_left,
                                                  passed_vertices_left, mid_route_left);
        node->right = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[1], visited_right,
                                                   passed_vertices_right, mid_route_right);
        return node;
    }

    /* In case there are no valid neighbors, return the node with no children 
    and add the visited route to the routes array in the tree attribute */
    index_routes_.push_back(visited);
    midpoint_routes_.push_back(mid_route);
    return node;
}
