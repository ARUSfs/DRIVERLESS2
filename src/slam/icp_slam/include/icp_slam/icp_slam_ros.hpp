#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/impl/centroid.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "PointXYZColorScore.h"
#include <common_msgs/CarState.h>

using namespace std;

class ICP_handle {
	private:
		bool has_map = false;
		int callback_iteration = 0;
		void send_position();
		ros::Time prev_t;
		float yaw_rate=0;
		float vx=0;
		float lap_count = 0;
		ros::Time lap_time;
		bool start_braking = false;
	public:
		ros::NodeHandle nh;
		ros::Subscriber perception_sub;
		ros::Subscriber state_sub;
		ros::Publisher map_publisher;
		ros::Publisher global_map_publisher;
		ros::Publisher lap_count_publisher;
		ros::Publisher finished_pub;
		// ros::Publisher slam_speed_publisher;

		std::string global_frame;
		std::string car_frame;

		bool restart_map;
		bool restart_map_at_origin;
		int restart_iterations;
		std::string mission;
		float braking_offset;

		tf2_ros::TransformBroadcaster br;

		pcl::PointCloud<PointXYZColorScore>::Ptr previous_map;
		pcl::PointCloud<PointXYZColorScore>::Ptr allp_clustered;

		Eigen::Matrix4f position;
		Eigen::Matrix4f prev_transformation;

		float score = 1.0;

		ICP_handle();
		void map_callback(sensor_msgs::PointCloud2);
		void state_callback(common_msgs::CarState);
};
