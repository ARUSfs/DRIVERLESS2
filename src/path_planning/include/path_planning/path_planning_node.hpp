#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <path_planning/delaunay_triangulation.hpp>
#include <common_msgs/msg/point2d.hpp>
#include <common_msgs/msg/triangle.hpp>
#include <common_msgs/msg/triangulation.hpp>
#include "path_planning/PointXYZColorScore.h"


class PathPlanning : public rclcpp::Node
{
    public:
        PathPlanning();
    private:
        std::string kPerceptionTopic;
        std::string kTriangulationTopic;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;
        rclcpp::Publisher<common_msgs::msg::Triangulation>::SharedPtr triangulation_pub_;
        void perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
        DelaunayTriangulation triangulation_class_;
        GEOM_FADE2D::Fade_2D triangulation_geom_;
};