#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "path_planning/delaunay_triangulation.hpp"
#include "CDT.h"
#include <path_planning/Triangulation.h>
#include <path_planning/CDTUtils.h>
#include <common_msgs/msg/point_xy.hpp>
#include <common_msgs/msg/simplex.hpp>
#include <common_msgs/msg/triangulation.hpp>


class PathPlanning : public rclcpp::Node
{
    public:
        PathPlanning();
    private:
        std::string kPerceptionTopic;
        std::string kTriangulationTopic;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;
        rclcpp::Publisher<common_msgs::msg::Triangulation>::SharedPtr triangulation_pub_;
        void perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg);
        CDT::Triangulation<double> triangulation_;
        
        
};