/**
 * @file simplex_node.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @date 12-11-2024
 * @brief 
 * 
 */

class SimplexNode{
    public:
        SimplexNode();
        SimplexNode(int key);
        int index;
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