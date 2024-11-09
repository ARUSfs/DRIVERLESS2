/**
 * @file generic_tree.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @date 01-11-2024
 * @brief Creates a tree structure to store information about neighbor triangles.
 * It also contain helper functions such as a search function.
 * 
 */

/**
 * @brief Class for the simlpex tree structure.
 * Nodes are indices of the triangles in the triangulation. Two nodes are connected if they share an edge.
 */
class generic_tree
{
public:
    /**
     * @brief Index of the triangle in the triangulation. int type.
     */
    int index;

    /**
     * @brief Right and left children of the node. Recursive structure for tree construction.
     */
    generic_tree *right, *left;

    /**
     * @brief Construct a new generic tree object.
     * @param key int index of the triangle.
     */
    generic_tree(int key);
};

generic_tree::generic_tree(int key)
{
    index = key;
    right = nullptr;
    left = nullptr;
}

bool tree_search (int key, generic_tree *root){
    if (root == nullptr) return false;
    if (root->index == key) return true;
    return tree_search(key, root->left) || tree_search(key, root->right);
}

