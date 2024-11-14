/**
 * @file simplex_node.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @date 12-11-2024
 * @brief 
 * 
 */

class SimplexNode{
    public:
        /**
         * @brief Default constructor for the SimplexNode class.
         * Constructs a new SimplexNode object with index 0 and nullptr children.
         */
        SimplexNode();

        /**
         * @brief Construct a new SimplexNode object with a given key.
         * @param key to assign to the node index.
         */
        SimplexNode(int key);

        /**
         * @brief index corresponding to the triangle in the triangulation.
         */
        int index;

        /**
         * @brief Children of the node.
         * Recursive structure, the children are also SimplexNode objects.
         */
        SimplexNode *right, *left;

};

SimplexNode::SimplexNode(int key){
    index = key;
    right = nullptr;
    left = nullptr;
}

SimplexNode::SimplexNode(){
    index = 0;
    right = nullptr;
    left = nullptr;}