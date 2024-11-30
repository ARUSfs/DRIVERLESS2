#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/state.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "ConeXYZColorScore.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/common/common.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/impl/centroid.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


class IcpSlam : public rclcpp::Node
{
	public:
		IcpSlam();
	private:
		bool has_map_ = false;
		int callback_iteration_ = 0;
		rclcpp::Time prev_t_;
		rclcpp::Time lap_time_;

		double vx_;
		double r_;

		pcl::PointCloud<ConeXYZColorScore>::Ptr previous_map_;
		pcl::PointCloud<ConeXYZColorScore>::Ptr allp_clustered_;

		Eigen::Matrix4f position_;
		Eigen::Matrix4f prev_transformation_;

		std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

		void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr map_msg);
		void state_callback(const common_msgs::msg::State::SharedPtr state_msg);
		void send_position();

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;
		rclcpp::Subscription<common_msgs::msg::State>::SharedPtr state_sub_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;

		// Parameters
		bool kRestartAtOrigin;
		int kRestartIterations;
};

