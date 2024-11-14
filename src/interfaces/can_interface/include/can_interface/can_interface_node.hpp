/**
 * @file can_interface_node.hpp
 * @brief Can Interface node header for ARUS Team Driverless pipeline
 */

#include <rclcpp/rclcpp.hpp>
#include <canlib.h>
#include <thread>
#include <iostream>

/**
 * @class CanInterface
 * @brief CanInterface class 
 * 
 * This class create the threads and the lines of CAN, and read them.
 */
class CanInterface : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the CanInterface class.
         */
        CanInterface(); 
        /**
         * @brief Function to know the status of the CAN
         * 
         * @return the error in case there is a problem with the driver you are connected to
         */
        void check_can(canStatus stat);
        
    private:
        /**
         * @brief Function which read the CAN line 1
         * 
         */
        void read_can0();

        /**
         * @brief Function which read the CAN line 1
         * 
         */
        void read_can1();
};