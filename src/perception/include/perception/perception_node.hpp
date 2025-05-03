/**
 * @file perception_node.hpp
 * @author Alejandro Vallejo Mayo (alejandro.vm.1805@gmail.com)
 * @brief Contains the class definition and the declaration of the methods used in the algorithm. 
 */


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "ground_filtering.h"
#include "clustering.h"
#include "cropping.h"
#include "scoring.h"
#include "accumulation.h"
#include "PointXYZColorScore.h"
#include "color_estimation.h"
#include "utils.h"
#include "common_msgs/msg/state.hpp"


/**
 * @class Perception
 * @brief Class containing the Perception node.
 */
class Perception : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new Perception object.
         */
        Perception();

    private:
        // Variables
        double vx;
        double vy;
        double yaw_rate;
        double dt;

        // Topics
        std::string kLidarTopic;
        std::string kStateTopic;

        // Parameters of cropping
        bool kCrop;
        double kMaxXFov;
        double kMaxYFov;
        double kMaxZFov;

        // Parameters of ground filtering
        double kThresholdGroundFilter;
        int kNumberSections;
        double kAngleThreshold;
        int kMinimumRansacPoints;

        // Parameters of clustering and filtering
        double kRadius;

        // Parameters of scroring
        double kThresholdScoring;
        
        // Parameters of coloring
        double kDistanceThreshold;
        double kColoringThreshold;

        // Parameters of debugging
        bool kDebug;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
        rclcpp::Subscription<common_msgs::msg::State>::SharedPtr state_sub_;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
        
        /**
         * @brief Callback function for the lidar topic.
         */
        void lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg);

        /**
        * @brief Callback function for the car state topic.
        */
        void state_callback(common_msgs::msg::State::SharedPtr state_msg);
};