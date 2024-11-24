/**
 * @file can_interface_node.cpp
 * @brief CanInterface node implementaion for ARUS Team Driverless pipeline
 */

#include "can_interface/can_interface_node.hpp"

/**
 * @class CanInterface
 * @brief CanInterface class 
 * 
 * This class initialize the canlib library, create a thread for each CAN line and create the handles.
 */ 
CanInterface::CanInterface() : Node("can_interface")
{

}

/**
 * @brief Fuction to import the CSV as a key and a vector, row by row.
 * 
 */
void CanInterface::read_csv()
{
    std::ifstream file("can_interface/data.csv");
    std::string line;
    std::map<std::string, std::vector<std::string>> dataMap;

    // Skip the first line (headers)
    if (std::getline(file, line)) {
        // The header line is read but not used
    }

    // Read the rest of the lines and process them
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string id;
        std::string value;
        std::vector<std::string> values;

        // Read the first column as the key (id)
        if (std::getline(ss, id, ',')) {
            // Read the rest of the columns and store them in the vector
            while (std::getline(ss, value, ',')) {
                values.push_back(value);
            }
        }

        // Insert the key and associated vector into the map
        dataMap[id] = values;
    }

    // Close the file
    file.close();
}


/**
 * @brief Function to know the status of the CAN
 * 
 * @return the error in case there is a problem with the driver you are connected to
 */
void CanInterface::check_can()
{

}

/**
 * @brief Function which read the CAN line 0
 * 
 */
void CanInterface::read_can0()
{   

}

/**
 * @brief Function which read the CAN line 1
 * 
 */
void CanInterface::read_can1()
{   

}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanInterface>());
    rclcpp::shutdown();
    return 0;
}
