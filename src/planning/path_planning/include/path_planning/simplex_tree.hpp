/**
 * @file simplex_tree.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @brief Creates a tree structure using information about neighbor triangles.
 */
#include <path_planning/simplex_node.hpp>
#include <iostream>
#include <CDT.h>
#include <Triangulation.h>
#include <CDTUtils.h>
#include "utils.hpp"


/**
 * @brief Class for the simplex tree structure.
 * Nodes are indices of the triangles in the triangulation. Two nodes are connected if they share an
 * edge (with more conditions).
 */
class SimplexTree {
    public:
    SimplexNode root_;

    pcl::PointCloud<ConeXYZColorScore> cones_cloud_;

    double angle_coeff_;
    double len_coeff_;
    double max_cost_;
    double min_cost_ = INFINITY;
    std::vector<ConeXYZColorScore> best_route_;
    std::vector<int> best_index_route_;
    std::vector<std::vector<ConeXYZColorScore>> ending_routes_;

    std::vector<std::vector<int>> index_routes_;

    std::vector<std::vector<ConeXYZColorScore>> midpoint_routes_;

    /**
     * @brief Construct a new Simplex Tree object with uninitialized values.
     * 
     */
    SimplexTree() = default;

    /**
     * @brief Construct a new simplex tree object and expand it.
     */
    SimplexTree(CDT::TriangleVec triangle_list, int first_tri_ind, std::vector<int> first_edge,
                pcl::PointCloud<ConeXYZColorScore> cones_cloud, std::vector<int> back_route_vertices, 
                double initial_angle, double angle_coeff, double len_coeff);

    /**
     * @brief Recursive function to create the tree structure.
     * From a triangle list and an index from which to start, it creates a tree structure containing all 
     * the routes through the triangles that share an edge. Left is the default path.
     */
    SimplexNode* create_tree_aux(CDT::TriangleVec triangle_list, int index, 
                                 std::vector<int> visited, std::vector<int> passed_vertices,
                                 std::vector<ConeXYZColorScore> mid_route, double route_cost,
                                 double prev_angle);
};


SimplexTree::SimplexTree(CDT::TriangleVec triangle_list, int first_tri_ind, std::vector<int> first_edge,
                         pcl::PointCloud<ConeXYZColorScore> cones_cloud, std::vector<int> passed_vertices, 
                         double initial_angle, double angle_coeff, double len_coeff) {
    cones_cloud_ = cones_cloud;
    angle_coeff_ = angle_coeff;
    len_coeff_ = len_coeff;
    CDT::Triangle first_tri = triangle_list[first_tri_ind]; // Get triangle from index
    
    CDT::NeighborsArr3 neighbors = first_tri.neighbors;  // Get neighbors of the triangle

    std::vector<int> visited = {first_tri_ind}; // Initialize visited array with the origin index
    passed_vertices.push_back(first_edge[0]); // Add the origin vertices to the passed vertices
    passed_vertices.push_back(first_edge[1]); 

    root_.index = first_tri_ind;                 // Set the root index to the origin index

    ConeXYZColorScore v0 = cones_cloud_.points[first_edge[0]];
    ConeXYZColorScore v1 = cones_cloud_.points[first_edge[1]];

    std::vector<ConeXYZColorScore> mid_route = {ConeXYZColorScore((v0.x+v1.x)/2,
                                        (v0.y+v1.y)/2, 0, UNCOLORED, 1)}; // Start at the origin

    double prev_angle = atan2(mid_route[0].y, mid_route[0].x); // Save the previous angle for the next iteration

    root_.left = SimplexTree::create_tree_aux(triangle_list, first_tri_ind, visited, passed_vertices,
        mid_route, 0.0, initial_angle);

} 


SimplexNode* SimplexTree::create_tree_aux(CDT::TriangleVec triangle_list, int index,
                                          std::vector<int> visited, std::vector<int> passed_vertices,
                                          std::vector<ConeXYZColorScore> mid_route, double route_cost,   
                                          double prev_angle){

    SimplexNode* node = new SimplexNode(index);
    if (midpoint_routes_.size() > 3 and route_cost > 600*len_coeff_){ 
        // Best route cost will never be higher than 500 m times the length coeff plus the min cost
        return node;
    }
    CDT::Triangle triangle = triangle_list[index];     // Get the triangle from the index
    CDT::NeighborsArr3 neighbors = triangle.neighbors; // Get the neighbors of the triangle
    std::vector<ConeXYZColorScore> neighbor_edge = {};
    std::vector<int> valid_neighbors = {};
    for (int i = 0; i<3; i++){
        if (neighbors[i]>triangle_list.size()) continue; 

        CDT::Triangle n_triangle = triangle_list[neighbors[i]];
        std::vector<ConeXYZColorScore> next_edge = {};

        bool finish_route = false;

        for (auto v: n_triangle.vertices){
            if (!in(v, triangle.vertices)){
                if (in(v, passed_vertices)){ // Check the neighbor doesn't contain any passed vertex
                    if (passed_vertices.size() > cones_cloud_.size()*0.5 && (v == passed_vertices[0] || v == passed_vertices[1])){ // If the first vertex is reached again, end the route 
                        finish_route = true;
                    } else {
                        next_edge.clear();
                        break;
                    }
                }
            } else {
                next_edge.push_back(cones_cloud_.points[v]);
            }
        }

        if (finish_route && next_edge.size() == 2){
            auto v = visited;
            v.push_back(neighbors[i]);
            index_routes_.push_back(v);
            auto m = mid_route;
            m.push_back(ConeXYZColorScore((next_edge[0].x+next_edge[1].x)/2,
                                          (next_edge[0].y+next_edge[1].y)/2, 0, UNCOLORED, 1));
            midpoint_routes_.push_back(m);
            ending_routes_.push_back(m);
            if (m.size() > 0.95*cones_cloud_.size()){
                route_cost -= 100;
            } 
            if (route_cost < min_cost_){
                min_cost_ = route_cost;
                best_route_ = mid_route;
                best_index_route_ = visited;
            }
            continue;
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
        ConeXYZColorScore prev_midpoint = mid_route.back();
        mid_route.push_back(neighbor_edge[0]);
        
        // Cost calculations
        double angle = atan2(neighbor_edge[0].y-prev_midpoint.y, neighbor_edge[0].x-prev_midpoint.x);
        double angle_diff = abs(angle-prev_angle);
        double corr_angle_diff = std::min(angle_diff, 2*M_PI-angle_diff);
        double prev_angle = angle; // Save the previous angle for the next iteration
        double len = distance(neighbor_edge[0], prev_midpoint);
        if (corr_angle_diff > M_PI/6){
            route_cost += angle_coeff_*(3*pow(corr_angle_diff-M_PI/6, 2)+1);
        }
        route_cost -= len_coeff_*len;


        CDT::Triangle next_triangle = triangle_list[valid_neighbors[0]];
        for (auto v: triangle.vertices){
            if (!in(v, next_triangle.vertices)){
                passed_vertices.push_back(v);
            }
        }

        node->left = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[0], visited, 
                                                  passed_vertices, mid_route, route_cost, prev_angle);
        return node;
    }
    else if (valid_neighbors.size() == 2){
        // Store the previous midpoint to calculate the angle between the following midpoints
        ConeXYZColorScore prev_midpoint = mid_route.back();

        // In case two neighbors are valid, create both children and return the node
        std::vector<int> visited_left = visited;
        std::vector<int> visited_right = visited;
        visited_left.push_back(valid_neighbors[0]);
        visited_right.push_back(valid_neighbors[1]);

        std::vector<ConeXYZColorScore> mid_route_left = mid_route;
        std::vector<ConeXYZColorScore> mid_route_right = mid_route;
        mid_route_left.push_back(neighbor_edge[0]);
        mid_route_right.push_back(neighbor_edge[1]);

        // Cost calculations
        double cost_left = route_cost;
        double angle_left = atan2(neighbor_edge[0].y-prev_midpoint.y, neighbor_edge[0].x-prev_midpoint.x);
        double angle_diff_left = abs(angle_left-prev_angle);
        double corr_angle_diff_left = std::min(angle_diff_left, 2*M_PI-angle_diff_left);
        double prev_angle_left = angle_left; // Save the previous angle for the next iteration
        double len_left = distance(neighbor_edge[0], prev_midpoint);
        if (corr_angle_diff_left > M_PI/6){
            cost_left += angle_coeff_*(3*pow(corr_angle_diff_left-M_PI/6, 2)+1);
        }
        cost_left -= len_coeff_*len_left;

        double cost_right = route_cost;
        double angle_right = atan2(neighbor_edge[1].y-prev_midpoint.y, neighbor_edge[1].x-prev_midpoint.x);
        double angle_diff_right = abs(angle_right-prev_angle);
        double corr_angle_diff_right = std::min(angle_diff_right, 2*M_PI-angle_diff_right);
        double prev_angle_right = angle_right; // Save the previous angle for the next iteration
        double len_right = distance(neighbor_edge[1], prev_midpoint);
        if (corr_angle_diff_right > M_PI/6){
            cost_right += angle_coeff_*(3*pow(corr_angle_diff_right-M_PI/6, 2)+1);
        }
        cost_right -= len_coeff_*len_right;


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
                                                  passed_vertices_left, mid_route_left, cost_left,
                                                  prev_angle_left);
        node->right = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[1], visited_right,
                                                   passed_vertices_right, mid_route_right, cost_right,
                                                   prev_angle_right);
        return node;
    }

    /* In case there are no valid neighbors, return the node with no children 
    and add the visited route to the routes array in the tree attribute */
    index_routes_.push_back(visited);
    midpoint_routes_.push_back(mid_route);
    if (route_cost < min_cost_){
        min_cost_ = route_cost;
        best_route_ = mid_route;
        best_index_route_ = visited;
    }
    return node;
}
