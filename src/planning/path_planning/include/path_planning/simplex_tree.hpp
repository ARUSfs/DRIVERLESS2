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
/**
 * @brief Class for the simlpex tree structure.
 * Nodes are indices of the triangles in the triangulation. Two nodes are connected if they share an edge.
 */
class SimplexTree {
    public:
    /**
     * @brief Right and left children of the node. Recursive structure for tree construction.
     */
    SimplexNode root;

    /**
     * @brief Array of arrays containing all the posible routes through the tree.
     */
    std::vector<std::vector<int>> index_routes;

    /**
     * @brief Construct a new generic tree object.
     * @param key int index of the triangle.
     */
    SimplexTree(CDT::TriangleVec triangle_list, int origin, std::vector<int> o_triangles);

    /**
     * @brief Recursive function to create the tree structure.
     * From a triangle list and an index from which to start, it creates a tree structure containing all 
     * the routes through the triangles that share an edge.
     * Left is the default path, right is created when there are two valid neighbors.
     * @param triangle_list triangulation.triangles object containing all the triangles.
     * @param index triangle index from which to start
     * @param visited array containing the visited triangle in the current route.
     * @return SimplexNode* pointer to the created tree.
     */
    SimplexNode* create_tree_aux(CDT::TriangleVec triangle_list, int index, std::vector<int> visited,
                                 std::vector<int> o_triangles);
};

SimplexTree::SimplexTree(CDT::TriangleVec triangle_list, int origin_ind, std::vector<int> o_triangles) {
    CDT::Triangle origin = triangle_list[origin_ind]; // Get triangle from index
    CDT::NeighborsArr3 neighbors = origin.neighbors;  // Get neighbors of the triangle

    std::vector<int> visited = {origin_ind}; // Initialize visited array with the origin index
    root.index = origin_ind;                 // Set the root index to the origin index

    // Filter neighbors to find which of them are valid
    for (int i = 0; i<3; i++){
        if ((neighbors[i]<=triangle_list.size()) and 
            (std::find(o_triangles.begin(),o_triangles.end(),neighbors[i])==o_triangles.end())){
            root.left = SimplexTree::create_tree_aux(triangle_list, neighbors[i], visited, o_triangles);
            break;
        }
    }

} 


SimplexNode* SimplexTree::create_tree_aux(CDT::TriangleVec triangle_list, int index,
                                          std::vector<int> visited, std::vector<int> o_triangles) {
    visited.push_back(index); // Add the current index to the visited array
    SimplexNode* node;        // Create an empty node
    node->index = index;      // Set the index of the node to the current index
    
    CDT::Triangle triangle = triangle_list[index];     // Get the triangle from the index
    CDT::NeighborsArr3 neighbors = triangle.neighbors; // Get the neighbors of the triangle
    
    // Perform the same filtering as in the constructor
    std::vector<int> valid_neighbors = {};
    for (int i = 0; i<3; i++){
        if ((neighbors[i]<=triangle_list.size()) and (std::find(visited.begin(),visited.end(),neighbors[i])==visited.end())
            and std::find(o_triangles.begin(),o_triangles.end(),neighbors[i])==o_triangles.end()){
            valid_neighbors.push_back(neighbors[i]);
        }
    }
    // Only change is that the node is returned when there is only one valid neighbor
    if (valid_neighbors.size() == 1){
        // In case only one neighbor is valid, create the left child and return the node
        node->left = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[0], visited, o_triangles);
        return node;
    }
    else if (valid_neighbors.size() == 2){
        // In case two neighbors are valid, create both children and return the node
        node->left = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[0], visited, o_triangles);
        node->right = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[1], visited, o_triangles);;
        return node;
    }
    /* In case there are no valid neighbors, return the node with no children 
    and add the visited route to the routes array in the tree attribute */
    index_routes.push_back(visited);
    return node;
}
