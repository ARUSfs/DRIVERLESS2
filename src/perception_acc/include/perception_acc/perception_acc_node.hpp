#define PCL_NO_PRECOMPILE

#include "PointXYZColorScore.h"
#include "PointXYZIRingTime.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include "common_msgs/msg/state.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <deque>
#include <omp.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "perception_acc/ground_filtering.h"
#include "perception_acc/cropping.h"
#include "perception_acc/clustering.h"
#include "perception_acc/scoring.h"
#include "perception_acc/color_estimation.h"
#include "perception_acc/accumulation.h"
#include "perception_acc/utils.h"

/**
 * @class Perception
 * @brief Class containing the Perception node.
 * Manages subscribers and publisher for the different topics used in the algorithm.
 * It also contains the callback functions for the subscribers.
 */
class Perception : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new Perception object.
         * This initializes the perception node, declares parameters, creates
         * the subscribers and publishers and contains all the necessary variables to
         * set up the algorithm.
         */
        Perception();

    private:
        //Parameters
        bool kCrop;
        double kMaxXFov;
        double kMaxYFov;
        double kMaxZFov;
        double kHFov;
        double kThresholdGroundFilter;
        int kNumberSections;
        double kAngleThreshold;
        int kNumberRings;
        int kNumberSectors;
        double kMaxRadius;
        double kRadius;
        int kMinimumRansacPoints;
        double kThresholdScoring;
        double kDistanceThreshold;
        double kColoringThreshold;
        double kAccumulationThreshold;
        int kBufferSize;
        bool kAccumulation_clouds;
        bool kAccumulation_clusters;
        bool kGlobalAccumulation;
        double kDistanceLidarToCoG;
        float kDownsampleSize;
        double kVoxelSize;
        int kCloudsMatched;
        std::deque<pcl::PointCloud<PointXYZIRingTime>> cloud_buffer_;

        std::shared_ptr<Eigen::Vector3f> prev_normal_;

        // Variables
        double x_;
        double y_;
        double yaw_;

        double avg_time_;
        int N_it_;

        //Subscriber
        std::string kLidarTopic;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
        std::string kStateTopic;

        //Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr acum_points_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

        // Nuevos miembros para tf
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        /**
         * @brief Get the position of the car in the world frame.
         * * This function uses the tf2 library to get the transform between the car and the world frame.
         * * It uses the tf_buffer_ to get the transform and then it extracts the position and orientation of the car.
         * 
         */
        void get_tf_position();

        /**
         * @brief Callback function for the lidar topic.
         * When the lidar topic recieves a message, this function is called and performs
         * all the necessary steps to generate the suitable point cloud.
         * @param lidar_msg The point cloud message received from the lidar.
         */
        void lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg);

        

        void get_clusters_centers_ransac(std::vector<pcl::PointIndices>& cluster_indices,
            pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& clusters_centers);


        void get_clusters_centers(std::vector<pcl::PointIndices>& cluster_indices,
            pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& cluster_centers);
            
        
        void reconstruction(pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_plane, pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered, 
            std::vector<pcl::PointIndices>& cluster_indices, std::vector<PointXYZColorScore> cluster_centers, 
            double radius);

        void filter_clusters(std::vector<pcl::PointIndices>& cluster_indices,
            pcl::PointCloud<PointXYZIRingTime>::Ptr cloud_filtered, std::vector<PointXYZColorScore>& clusters_centers);
            
};