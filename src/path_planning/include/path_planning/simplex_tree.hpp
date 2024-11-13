/**
 * @file generic_tree.hpp
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
class SimplexTree
{
public:
    /**
     * @brief Right and left children of the node. Recursive structure for tree construction.
     */
    SimplexNode root;

    /**
     * @brief Array of arrays containing all the posible routes through the tree.
     */
    std::vector<std::vector<int>> routes;

    /**
     * @brief Construct a new generic tree object.
     * @param key int index of the triangle.
     */
    SimplexTree(CDT::TriangleVec triangle_list, CDT::Triangulation<double>::V2dVec vertices, 
int origin, std::vector<int> o_triangles);

    SimplexNode* create_tree_aux(CDT::TriangleVec triangle_list, int index, std::vector<int> visited);
};

SimplexTree::SimplexTree(CDT::TriangleVec triangle_list, CDT::Triangulation<double>::V2dVec vertices, 
int origin_ind, std::vector<int> o_triangles){
    CDT::Triangle origin = triangle_list[origin_ind];
    CDT::NeighborsArr3 neighbors = origin.neighbors;
    std::vector<int> valid_neighbors = {};
    for (int i = 0; i<3; i++){
        if ((neighbors[i]<=triangle_list.size()) and 
        (std::find(o_triangles.begin(),o_triangles.end(),neighbors[i])==o_triangles.end())){
            valid_neighbors.push_back(neighbors[i]);
        }
    }
    std::vector<int> visited = {origin_ind};
    root.index = origin_ind;
    if (valid_neighbors.size() == 1){
        root.left = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[0], visited);
    } else if (valid_neighbors.size() == 2){
        std::cout << "Algo mal" << std::endl;
        root.left = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[0], visited);
        root.right = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[1], visited);
    }
} 


SimplexNode* SimplexTree::create_tree_aux(CDT::TriangleVec triangle_list, int index, std::vector<int> visited){
    visited.push_back(index);
    SimplexNode* node;
    node->index = index;
    CDT::Triangle triangle = triangle_list[index];
    CDT::NeighborsArr3 neighbors = triangle.neighbors;
    std::vector<int> valid_neighbors = {};
    for (int i = 0; i<3; i++){
        if ((neighbors[i]<=triangle_list.size()) and (std::find(visited.begin(),visited.end(),neighbors[i])==visited.end())){
            valid_neighbors.push_back(neighbors[i]);
        }
    }
    if (valid_neighbors.size() == 1){
        node->left = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[0], visited);
        return node;
    }
    else if (valid_neighbors.size() == 2){
        node->left = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[0], visited);
        node->right = SimplexTree::create_tree_aux(triangle_list, valid_neighbors[1], visited);
        return node;
    }
    routes.push_back(visited);
    return node;
}

// void print_tree(SimplexTree *root){
//     if (root == nullptr) {
//         std::cout<< "fin de rama" <<std::endl;
//         return;
//     }
//     std::cout << root->index << std::endl;
//     print_tree(root->left);
//     print_tree(root->right);
// }

// bool tree_search (int key, SimplexTree *root){
//     if (root == nullptr) return false;
//     if (root->index == key) return true;
//     return tree_search(key, root->left) || tree_search(key, root->right);
// }
