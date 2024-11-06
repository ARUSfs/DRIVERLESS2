#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/state.hpp"

class TrajectoryOptimization : public rclcpp::Node
{
    public:

        TrajectoryOptimization();

    private:

        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr arussim_sub_;
        rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

        void arussim_callback(common_msgs::msg::Trajectory::SharedPtr assim_msg);
        void car_state_callback(common_msgs::msg::State::SharedPtr car_state_msg);

};