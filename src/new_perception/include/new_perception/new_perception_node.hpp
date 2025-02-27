/**
 * @file new_perception_node.hpp
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Header file for the NewPerception node.
 * Contains the class definition and the declaration of the methods 
 * used in the algorithm. 
 * @version 0.1
 * @date 27-2-2025
 */

#include <rclcpp/rclcpp.hpp>

/**
 * @class NewPerception
 * @brief Class containing the NewPerception node.
 * Manages subscribers and publisher for the different topics used in the algorithm.
 * It also contains the callback functions for the subscribers.
 */
class NewPerception : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a NewPerception object.
         * This initializes the new perception node, declares parameters, creates
         * the subscribers and publishers and contains all the necessary variables to
         * set up the algorithm.
         */
        NewPerception();
};