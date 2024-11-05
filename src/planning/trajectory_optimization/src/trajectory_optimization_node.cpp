#include "trajectory_optimization/trajectory_optimization_node.hpp"

TrajectoryOptimization::TrajectoryOptimization() : Node("trajectory_optimization")
{

}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryOptimization>());
    rclcpp::shutdown();
    return 0;
}