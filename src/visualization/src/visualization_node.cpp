#include "visualization/visualization_node.hpp"

Visualization::Visualization() : Node("visualization")
{

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualization>());
    rclcpp::shutdown();
    return 0;
}