#include "trajectory_optimization/trajectory_optimization_node.hpp"

TrajectoryOptimization::TrajectoryOptimization() : Node("trajectory_optimization")
{
    arussim_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        "/arussim_interface/fixed_trajectory", 10, std::bind(&TrajectoryOptimization::arussim_callback, this, std::placeholders::_1));
    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
        "/car_state/state", 1, std::bind(&TrajectoryOptimization::car_state_callback, this, std::placeholders::_1));
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>("/trajectory_optimizer/trajectory", 10);
}

void TrajectoryOptimization::arussim_callback(common_msgs::msg::Trajectory::SharedPtr assim_msg){

}

void TrajectoryOptimization::car_state_callback(common_msgs::msg::State::SharedPtr car_state_msg){

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryOptimization>());
    rclcpp::shutdown();
    return 0;
}