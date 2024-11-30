#define PCL_NO_PRECOMPILE

#include "icp_slam/icp_slam_node.hpp"


IcpSlam::IcpSlam() : Node("icp_slam")
{	

	this->declare_parameter("restart_at_origin", true);
	this->declare_parameter("restart_iterations", -1);
	this->get_parameter("restart_at_origin", kRestartAtOrigin);
	this->get_parameter("restart_iterations", kRestartIterations);

  	previous_map_ = pcl::PointCloud<ConeXYZColorScore>::Ptr(new pcl::PointCloud<ConeXYZColorScore>);
	allp_clustered_ = pcl::PointCloud<ConeXYZColorScore>::Ptr(new pcl::PointCloud<ConeXYZColorScore>);

    position_ = Eigen::Matrix4f::Identity(4, 4);
	prev_transformation_ = Eigen::Matrix4f::Identity(4, 4);

	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
	
	state_sub_ = this->create_subscription<common_msgs::msg::State>("/car_state/state", 10, std::bind(&IcpSlam::state_callback, this, std::placeholders::_1));
    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/arussim/perception", 10, std::bind(&IcpSlam::map_callback, this, std::placeholders::_1));
    
    map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/map", 10);


}

void IcpSlam::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr map_msg)
{
	pcl::PointCloud<ConeXYZColorScore>::Ptr new_map(new pcl::PointCloud<ConeXYZColorScore>);
	pcl::fromROSMsg(*map_msg, *new_map);

	if (new_map->empty()){
		return;
	}

	if(!has_map_){
		prev_t_ = this->now();
		*allp_clustered_ = *new_map;
		*previous_map_ = *new_map;
		has_map_ = true;
		return;
	}
	
	//get ICP transformation
	pcl::PointCloud<ConeXYZColorScore>::Ptr map_in_position = pcl::PointCloud<ConeXYZColorScore>::Ptr(new pcl::PointCloud<ConeXYZColorScore>);

	pcl::transformPointCloud(*new_map, *map_in_position, position_);
	
	pcl::IterativeClosestPoint<ConeXYZColorScore, ConeXYZColorScore> icp;
	icp.setInputSource(map_in_position);
	if(callback_iteration_ < 10)
		icp.setInputTarget(previous_map_);
	else
		icp.setInputTarget(allp_clustered_);
	icp.setMaximumIterations(5);
	icp.setEuclideanFitnessEpsilon(0.005);
	icp.setTransformationEpsilon(1e-5);
	
	pcl::PointCloud<ConeXYZColorScore>::Ptr registered_map = pcl::PointCloud<ConeXYZColorScore>::Ptr(new pcl::PointCloud<ConeXYZColorScore>);
	icp.setMaxCorrespondenceDistance (1.0);
	icp.align(*registered_map);
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	


	//get transformation estimation from odometry
	float dt = this->now().seconds()-prev_t_.seconds();
	float dx = dt*vx_;
	float dyaw = dt*r_; 

	float yaw = (float)-atan2(position_.coeff(0, 1), position_.coeff(0,0));
	Eigen::Matrix4f estimation = Eigen::Matrix4f::Identity();
	estimation(0,3)=dx*cos(yaw+dyaw)+position_(0,3)-cos(dyaw)*position_(0,3)+sin(dyaw)*position_(1,3);
	estimation(1,3)=dx*sin(yaw+dyaw)+position_(1,3)-sin(dyaw)*position_(0,3)-cos(dyaw)*position_(1,3);
	estimation(0,0)=cos(dyaw);
	estimation(1,0)=sin(dyaw);
	estimation(0,1)=-sin(dyaw);
	estimation(1,1)=cos(dyaw);


	//get transformation distance
	float tx = transformation.coeff(0,0)*position_.coeff(0,3) + transformation.coeff(0,1)*position_.coeff(1,3) + transformation.coeff(0,3) - position_.coeff(0,3);
	float ty = transformation.coeff(1,0)*position_.coeff(0,3) + transformation.coeff(1,1)*position_.coeff(1,3) + transformation.coeff(1,3) - position_.coeff(1,3);
	float dist = sqrt(tx*tx + ty*ty);


	//sigmoidal weighting of transformations
	float tyaw = (float)-atan2(transformation.coeff(0, 1), transformation.coeff(0,0)); 
	float w;
	if(dist == 0 || std::abs(tyaw-dyaw)>0.1)
		w = 1;
	else
		w = -1 + 2.0/(1.0 + std::exp(-(5*std::abs(dist-dx)+10*std::abs(tyaw-dyaw))));
	// std::cout << "ICP confidence: " << 1-w << std::endl;
	
	//update position
	prev_transformation_ = (estimation*w + transformation*(1-w));
	float prev_x = position_.coeff(0,3);
	float prev_y = position_.coeff(1,3);
	position_ = prev_transformation_*position_;
	prev_t_ = this->now();
	

	//update map
	pcl::PointCloud<ConeXYZColorScore>::Ptr registered_map2 = pcl::PointCloud<ConeXYZColorScore>::Ptr(new pcl::PointCloud<ConeXYZColorScore>);
	pcl::transformPointCloud(*new_map, *registered_map2, position_);
	*previous_map_ += *registered_map2;



	//map clustering
	pcl::search::KdTree<ConeXYZColorScore>::Ptr tree (new pcl::search::KdTree<ConeXYZColorScore>);
	tree->setInputCloud(previous_map_);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<ConeXYZColorScore> ec;
	ec.setClusterTolerance (1.5);
	ec.setMinClusterSize (3);
	ec.setMaxClusterSize (100000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (previous_map_);
	ec.extract (cluster_indices);

	pcl::PointCloud<ConeXYZColorScore>::Ptr clustered_points (new pcl::PointCloud<ConeXYZColorScore>);
	for(const auto& cluster : cluster_indices) {
		ConeXYZColorScore centro;
		centro.x = 0;
		centro.y = 0;
		centro.z = 0;
		centro.color = 0;
		centro.score = 1;
		for (const auto& idx : cluster.indices) {
			centro.x += (*previous_map_)[idx].x;
			centro.y += (*previous_map_)[idx].y;
			centro.z = 0;

		}
		centro.x /= cluster.indices.size();
		centro.y /= cluster.indices.size();
		for (const auto& idx : cluster.indices) {
			(*previous_map_)[idx].x = centro.x;
			(*previous_map_)[idx].y = centro.y;
		}

		clustered_points->push_back(centro);

	}
	*allp_clustered_ = *clustered_points;



	if(callback_iteration_ % 10 == 9) {
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<ConeXYZColorScore> ec;
		ec.setClusterTolerance (1.5);
		ec.setMinClusterSize (6);
		ec.setMaxClusterSize (200);
		ec.setSearchMethod (tree);
		ec.setInputCloud (previous_map_);
		ec.extract (cluster_indices);



		pcl::PointCloud<ConeXYZColorScore>::Ptr clustered_points (new pcl::PointCloud<ConeXYZColorScore>);
		pcl::PointCloud<ConeXYZColorScore>::Ptr mapa_global (new pcl::PointCloud<ConeXYZColorScore>);
		for(const auto& cluster : cluster_indices) {
			int j = 0;
			ConeXYZColorScore cono;
			cono.x = (*previous_map_)[cluster.indices[0]].x;
			cono.y = (*previous_map_)[cluster.indices[0]].y;
			cono.z = 0;
			cono.color = 0;
			cono.score = 1;
			for (const auto& idx : cluster.indices) {
								// This is a botch that works but should be changed. PCL ICP doesn't sopport weighted points,
								// so the more reliable we consider a point to be, the more that are placed on the exact same
								// coordinates (limited in this case to 50).
				if(j < 10)
					clustered_points->push_back((*previous_map_)[idx]);
				else
					break;
				j++;
			}
			mapa_global->push_back(cono);
		}

		*previous_map_ = *clustered_points;
	}

	if(kRestartIterations != -1 && callback_iteration_>kRestartIterations){
		has_map_ = false;
		callback_iteration_ = 0;
	}

	sensor_msgs::msg::PointCloud2 new_map_msg;

	pcl::toROSMsg(*allp_clustered_, new_map_msg);
	new_map_msg.header.frame_id = "/arussim/world";
	map_publisher_->publish(new_map_msg);

	callback_iteration_++;

}

void IcpSlam::state_callback(const common_msgs::msg::State::SharedPtr state_msg)
{
    vx_ = state_msg->vx;
    r_ = state_msg->r;

    send_position();
}

void IcpSlam::send_position() {
	float dt = this->now().seconds()-prev_t_.seconds();
	geometry_msgs::msg::TransformStamped transformSt;
	transformSt.header.stamp = this->now();
	transformSt.header.frame_id = "/arussim/world";
	transformSt.child_frame_id = "/slam/vehicle";
	tf2::Quaternion q;
	float ang = (float)-atan2(position_.coeff(0, 1), position_.coeff(0,0)) + r_*dt;
	transformSt.transform.translation.x = position_.coeff(0,3)+vx_*dt*cos(ang);
	transformSt.transform.translation.y = position_.coeff(1,3)+vx_*dt*sin(ang);
	q.setRPY(0, 0, ang);
	transformSt.transform.rotation.x = q.x();
	transformSt.transform.rotation.y = q.y();
	transformSt.transform.rotation.z = q.z();
	transformSt.transform.rotation.w = q.w();

	tf_broadcaster_->sendTransform(transformSt);

	if (position_.coeff(0,3)*position_.coeff(0,3)+position_.coeff(1,3)*position_.coeff(1,3) < 10){
		if(this->now().seconds()-lap_time_.seconds() > 20 && vx_>1 ){
			if(kRestartAtOrigin){
				has_map_ = false;
			}
		}
		lap_time_ = this->now();
	} 

	
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpSlam>());
  rclcpp::shutdown();
  return 0;
}