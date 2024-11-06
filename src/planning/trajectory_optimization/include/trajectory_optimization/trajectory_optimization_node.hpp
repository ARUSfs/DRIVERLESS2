#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/state.hpp"

class TrajectoryOptimization : public rclcpp::Node
{
    public:

        TrajectoryOptimization();

    private:

        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr optimized_trajectory_pub_;

        void trajectory_callback(common_msgs::msg::Trajectory::SharedPtr trajectory_msg);
        void car_state_callback(common_msgs::msg::State::SharedPtr car_state_msg);

};