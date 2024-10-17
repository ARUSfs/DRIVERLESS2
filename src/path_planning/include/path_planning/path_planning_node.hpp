#include <CGAL/Triangle_2.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PathPlanningNode : public rclcpp::Node
{
    public:
        PathPlanningNode();
    private:
        std::string kPerceptionTopic;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;

        void perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};