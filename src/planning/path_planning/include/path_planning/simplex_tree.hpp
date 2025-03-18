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

    double angle_coeff_;
    double len_coeff_;
    double max_cost_;
    double min_cost_ = INFINITY;
    std::vector<ConeXYZColorScore> best_route_;
    std::vector<int> best_index_route_;
    std::vector<std::vector<ConeXYZColorScore>> ending_routes_;

    /**
     * @brief Array of arrays containing all the posible routes through the tree.
     */
    std::vector<std::vector<int>> index_routes_;

    /**
     * @brief Array of arrays containing the midpoints of the edges in the routes.
     * 
     */
    std::vector<std::vector<ConeXYZColorScore>> midpoint_routes_;

    /**
     * @brief Construct a new Simplex Tree object with uninitialized values.
     * 
     */
    SimplexTree() = default;

    /**
     * @brief Construct a new generic tree object.
     * @param triangle_list triangulation.triangles object containing all the triangles.
     * @param origin index of the origin triangle.
     * @param orig_vertex index of the origin vertex.
     * @param cones_cloud pcl::PointCloud object containing the cones in the map.
     * @param yaw yaw of the car.
     * @param angle_coeff coefficient for the angle cost.
     * @param len_coeff coefficient for the length cost.
     */
    SimplexTree(CDT::TriangleVec triangle_list, int origin, int orig_vertex, 
                pcl::PointCloud<ConeXYZColorScore> cones_cloud, double yaw, double angle_coeff,
                double len_coeff);

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
                                 std::vector<ConeXYZColorScore> mid_route, double route_cost,
                                 double prev_angle);
};

SimplexTree::SimplexTree(CDT::TriangleVec triangle_list, int origin_ind, int orig_vertex,
                         pcl::PointCloud<ConeXYZColorScore> cones_cloud, double yaw, double angle_coeff, 
                         double len_coeff) {
    cones_cloud_ = cones_cloud;
    angle_coeff_ = angle_coeff;
    len_coeff_ = len_coeff;
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

            // Add first edge to the mid_route array
            std::vector<ConeXYZColorScore> edge = {};
            for (auto v: triangle_list[neighbors[i]].vertices){
                if (in(v, origin.vertices)){
                    edge.push_back(cones_cloud_.points[v]);
                }
            }

            mid_route.push_back(ConeXYZColorScore((edge[0].x+edge[1].x)/2,
                                                  (edge[0].y+edge[1].y)/2, 0, UNCOLORED, 1));            
            
            // Calculate cost of the first edge
            double route_cost = 0;
            double angle = atan2(mid_route[1].y-mid_route[0].y, mid_route[1].x-mid_route[0].x);
            double angle_diff = abs(angle-yaw);
            double corr_angle_diff = std::min(angle_diff, 2*M_PI-angle_diff);
            double len = distance(mid_route[1], mid_route[0]);
            if (corr_angle_diff > M_PI/6){
                route_cost += angle_coeff_*(3*pow(corr_angle_diff-M_PI/6, 2)+1);
            }
            route_cost -= len_coeff_*len;

            visited.push_back(neighbors[i]); // Add the valid neighbors to the visited array
            root_.left = SimplexTree::create_tree_aux(triangle_list, neighbors[i], visited, passed_vertices,
                                                      mid_route, route_cost, angle);
            break;
        }
    }

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
                    if (visited.size()>20 && v == passed_vertices[0]){ // If the first vertex is reached again, end the route 
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
            visited.push_back(neighbors[i]);
            index_routes_.push_back(visited);
            mid_route.push_back(ConeXYZColorScore((next_edge[0].x+next_edge[1].x)/2,
                                                      (next_edge[0].y+next_edge[1].y)/2, 0, UNCOLORED, 1));
            midpoint_routes_.push_back(mid_route);
            ending_routes_.push_back(mid_route);
            if (mid_route.size() > 0.95*cones_cloud_.size()){
                route_cost -= 100;
            } 
            if (route_cost < min_cost_){
                min_cost_ = route_cost;
                best_route_ = mid_route;
                best_index_route_ = visited;
            }
            return node;
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
