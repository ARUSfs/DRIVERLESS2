#include "graph_slam/graph_slam_node.hpp"
#include <g2o/core/robust_kernel_impl.h>

GraphSlam::GraphSlam() : Node("graph_slam")
{ 

    this->declare_parameter("finish_line_offset", 0.0);
    this->declare_parameter("track_width", 3.0);
    this->declare_parameter("min_lap_distance", 30.0);
    this->declare_parameter("write_csv", false);
    this->declare_parameter("max_pose_edges", 10000);
    this->declare_parameter("max_landmark_edges", 10000);
    this->declare_parameter("verbose", false);
    this->declare_parameter("pos_lidar_x", 1.5);
    this->declare_parameter("perception_topic", "/perception/map");
    this->get_parameter("finish_line_offset", kFinishLineOffset);
    this->get_parameter("track_width", kTrackWidth);
    this->get_parameter("min_lap_distance", kMinLapDistance);
    this->get_parameter("write_csv", kWriteCSV);
    this->get_parameter("max_pose_edges", kMaxPoseEdges);
    this->get_parameter("max_landmark_edges", kMaxLandmarkEdges);
    this->get_parameter("verbose", kVerbose);
    this->get_parameter("pos_lidar_x", kPosLidarX);
    this->get_parameter("perception_topic", kPerceptionTopic);

    // TODO: test other solvers
    using SlamBlockSolver  = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
    using SlamLinearSolver = g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>;
    optimizer_.setVerbose(false);
    auto linearSolver = std::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(true);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    solver->setUserLambdaInit(10);
    optimizer_.setAlgorithm(solver);

    g2o::VertexSE2* init_vertex = new g2o::VertexSE2();
    init_vertex->setId(1); // Use odd ids for pose vertices
    init_vertex->setEstimate(g2o::SE2(0,0,0));
    init_vertex->setFixed(true);
    pose_vertices_.push_back(init_vertex);
    optimizer_.addVertex(init_vertex);
    
    prev_t_ = this->now();
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


    Q(0, 0) *= x_noise_;
    Q(1, 1) *= y_noise_;
    Q(2, 2) *= yaw_noise_;
    R(0, 0) *= perception_noise_;
    R(1, 1) *= perception_noise_;

    // Create callback groups to allow parallel processing of collector and optimizer
    // Callbacks in the same group are processed sequentially, 
    // while callbacks in different groups are processed in parallel
    collector_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    optimizer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // SubscriptionOptions object is necessary to specify the callback group in Subscribers
    auto collector_options = rclcpp::SubscriptionOptions();
    collector_options.callback_group = collector_callback_group_;

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/graph_slam/marker", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/map", 10);
    final_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/final_map", 10);
    lap_count_pub_ = this->create_publisher<std_msgs::msg::Int16>("/slam/lap_count", 10);

    state_sub_ = this->create_subscription<common_msgs::msg::State>("/car_state/state", 10, 
                    std::bind(&GraphSlam::state_callback, this, std::placeholders::_1), collector_options);
    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(kPerceptionTopic, 10,
                    std::bind(&GraphSlam::perception_callback, this, std::placeholders::_1), collector_options);           
    // TODO: run optimizer depending on the number of edges/itereations
    optimizer_timer_ = this->create_wall_timer(std::chrono::milliseconds(300),
                    std::bind(&GraphSlam::optimizer_callback, this), optimizer_callback_group_);
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////// CALLBACKS /////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void GraphSlam::state_callback(const common_msgs::msg::State::SharedPtr msg)
{   
    // Update pose estimation from odometry
    double phi = vehicle_pose_(2);
    B.block(0, 0, 2, 2) << std::cos(phi), -std::sin(phi), std::sin(phi), std::cos(phi);
    u << msg->vx, msg->vy, msg->r;
    vehicle_pose_ += B * u * dt; // TODO check phi in range [-pi, pi]
    driven_distance_ += (B * u * dt).norm();

    // Add the odometry to the buffer to re-predict the pose after optimization
    odom_buffer_.push_back(u);
    
    if (vehicle_pose_(2) > M_PI) {
        vehicle_pose_(2) -= 2*M_PI;
    } else if (vehicle_pose_(2) < -M_PI) {
        vehicle_pose_(2) += 2*M_PI;
    }

    prev_t_ = this->now();
    // Publish the vehicle pose as a tf
    send_tf();
    // Check if the vehicle has crossed the finish line
    check_finish_line();

    // Add a new pose vertex
    g2o::VertexSE2* pose_vertex = new g2o::VertexSE2();
    pose_vertex->setId(2*pose_vertices_.size()+1); // Use odd ids for pose vertices
    pose_vertex->setEstimate(g2o::SE2(vehicle_pose_));
    pose_vertices_.push_back(pose_vertex);
    pose_vertices_to_add_.push_back(pose_vertex);

    if (pose_vertices_.size() < 2) {
        return;
    }

    // Get the previous pose vertex
    g2o::VertexSE2* prev_pose_vertex = pose_vertices_[pose_vertices_.size()-2];

    // Add an edge between the previous pose vertex and the new pose vertex
    g2o::EdgeSE2* pose_edge = new g2o::EdgeSE2();
    pose_edge->vertices()[0] = prev_pose_vertex;
    pose_edge->vertices()[1] = pose_vertex;
    // The weight of the edge is the transform between the two poses
    g2o::SE2 transform = prev_pose_vertex->estimate().inverse() * pose_vertex->estimate();
    pose_edge->setMeasurement(transform);
    // The information matrix is the inverse of the covariance matrix
    pose_edge->setInformation(Q.inverse());
    g2o::RobustKernelHuber* rkPose = new g2o::RobustKernelHuber();
    pose_edge->setRobustKernel(rkPose);
    rkPose->setDelta(0.3);
    pose_edges_.push_back(pose_edge);
    edges_to_add_.push_back(pose_edge);
}

void GraphSlam::perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{   
    if (pose_vertices_.size() == 0) {
	return;
    }

    std::vector<Landmark> observed_landmarks;
    std::vector<Landmark> unmatched_landmarks;

    g2o::VertexSE2* last_pose_vertex = pose_vertices_.back();
    pcl::PointCloud<ConeXYZColorScore> cloud;
    pcl::fromROSMsg(*msg, cloud);
   
    for (int i = 0; i < msg->width; i++) {
        //Extract the cone position
        ConeXYZColorScore cone = cloud.points[i];

        observed_landmarks.push_back(Landmark(Eigen::Vector2d(cone.x+kPosLidarX, cone.y), vehicle_pose_));
    }

    if (observed_landmarks.size() == 0) {
        return;
    }
    
    DA.match_observations(observed_landmarks, unmatched_landmarks);
    if(!map_fixed_){
        for (auto landmark : unmatched_landmarks) {
            landmark.id_ = 2*DA.map_.size(); // Use even ids for landmark vertices
            Landmark* new_landmark = new Landmark(landmark); // Copy the landmark to avoid memory issues
            DA.map_.push_back(new_landmark);

            // Add a new landmark vertex 
            g2o::VertexPointXY* landmark_vertex = new g2o::VertexPointXY();
            landmark_vertex->setId(landmark.id_);
            landmark_vertex->setEstimate(landmark.world_position_);
            landmark_vertices_.push_back(landmark_vertex);
            landmark_vertices_to_add_.push_back(landmark_vertex);

            // Add an edge between the last pose vertex and the new landmark vertex 
            g2o::EdgeSE2PointXY* edge = new g2o::EdgeSE2PointXY();
            edge->vertices()[0] = last_pose_vertex;
            edge->vertices()[1] = landmark_vertex;
            edge->setMeasurement(landmark.local_position_);
            edge->setInformation(R.inverse());
            g2o::RobustKernelHuber* rkLandmark = new g2o::RobustKernelHuber();
            edge->setRobustKernel(rkLandmark);
            rkLandmark->setDelta(0.3);
            landmark_edges_.push_back(edge);
            edges_to_add_.push_back(edge);
        }
    }
    

    for (auto landmark : observed_landmarks) {
        if (landmark.id_ == Landmark::UNMATCHED_ID) {
            continue;
        }
        // Add an edge between the last pose vertex and the matched landmark vertex
        g2o::VertexPointXY* landmark_vertex = landmark_vertices_[landmark.id_/2];
        g2o::EdgeSE2PointXY* edge = new g2o::EdgeSE2PointXY();
        edge->vertices()[0] = last_pose_vertex;
        edge->vertices()[1] = landmark_vertex;
        edge->setMeasurement(landmark.local_position_);
        edge->setInformation(R.inverse());
        g2o::RobustKernelHuber* rkLandmark = new g2o::RobustKernelHuber();
        edge->setRobustKernel(rkLandmark);
        rkLandmark->setDelta(0.3);
        landmark_edges_.push_back(edge);
        edges_to_add_.push_back(edge);
    }

    publish_map();
}

void GraphSlam::optimizer_callback(){
    update_pose_predictions();

    fill_graph();

    optimizer_.initializeOptimization();

    double t0 = this->now().seconds();
    optimizer_.optimize(10);
    if(kVerbose){
        RCLCPP_INFO(this->get_logger(), "---------------------------------");
        RCLCPP_INFO(this->get_logger(), "Number of vertices: %d", optimizer_.activeVertices().size());
        RCLCPP_INFO(this->get_logger(), "Number of edges: %d", optimizer_.activeEdges().size());
        RCLCPP_INFO(this->get_logger(), "Optimization time: %f", this->now().seconds() - t0);
    }

    update_data_association_map();
    
}


/////////////////////////////////////////////////////////////////////////
///////////////////////////// FUNCTIONS /////////////////////////////////
/////////////////////////////////////////////////////////////////////////


// Update the world position of the landmarks in the data association map after optimization
// Landmarks must be pointers to be updated
void GraphSlam::update_data_association_map(){
    for (Landmark* landmark : DA.map_){
        for (auto vertex : optimizer_.activeVertices()) {
            g2o::VertexPointXY* landmark_vertex = dynamic_cast<g2o::VertexPointXY*>(vertex);
            if (landmark_vertex != nullptr && landmark_vertex->id() == landmark->id_) {
                landmark->world_position_ = landmark_vertex->estimate();
            }
        }
    }
}


void GraphSlam::update_pose_predictions(){

    if(last_optimized_pose_ == nullptr){
        return;
    }

    std::vector<Eigen::Vector3d> updated_poses;
    Eigen::Vector3d updated_pose = last_optimized_pose_->estimate().toVector();
    updated_poses.push_back(updated_pose);

    int index = last_optimized_pose_->id();
    Eigen::Matrix3d B2 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d u2 = Eigen::Vector3d::Zero();

    // Re-predict the pose for each odometry measurement in the buffer
    for (auto u2: odom_buffer_){
        double phi = updated_pose(2);
        B2.block(0, 0, 2, 2) << std::cos(phi), -std::sin(phi), std::sin(phi), std::cos(phi);
        updated_pose += B2 * u2 * dt;

        updated_poses.push_back(updated_pose);
    }

    // Update the pose vertices with the new predictions
    for(auto pose_vertex: pose_vertices_to_add_){
        pose_vertex->setEstimate(g2o::SE2(updated_poses[(pose_vertex->id() - index)/2]));
    }
    
    odom_buffer_.clear();

    // Update the vehicle pose with the last prediction
    double dx = updated_pose(0)-vehicle_pose_(0);
    double dy = updated_pose(1)-vehicle_pose_(1);
    double dyaw = updated_pose(2)-vehicle_pose_(2);
    if (dyaw > M_PI) {
        dyaw -= 2*M_PI;
    } else if (dyaw < -M_PI) {
        dyaw += 2*M_PI;
    }
    double max_offset = 1;
    double max_yaw_offset = M_PI/18;
    vehicle_pose_(0) += dx*std::min(1.0, max_offset/std::sqrt(dx*dx+dy*dy));
    vehicle_pose_(1) += dy*std::min(1.0, max_offset/std::sqrt(dx*dx+dy*dy));
    vehicle_pose_(2) += dyaw*std::min(1.0, max_yaw_offset/std::abs(dyaw));
}


void GraphSlam::fill_graph(){
    // Get last optimized pose from which predictions will be made
    last_optimized_pose_ = pose_vertices_.back();

    // Add vertices and edges to the optimizer
    for (auto vertex: pose_vertices_to_add_){
        optimizer_.addVertex(vertex);
    }
    pose_vertices_to_add_.clear();
    for (auto vertex: landmark_vertices_to_add_){
        optimizer_.addVertex(vertex);
    }
    landmark_vertices_to_add_.clear();
    for (auto edge: edges_to_add_){
        optimizer_.addEdge(edge);
    }
    edges_to_add_.clear();

    // Deactivate edges exceding the maximum
    if(pose_edges_.size()-pose_edges_deactivated_ > kMaxPoseEdges){
        for (int i=pose_edges_deactivated_; i<pose_edges_.size()-kMaxPoseEdges; i++){
            pose_edges_[i]->setLevel(2); // Deactivate edge
        }
        pose_edges_deactivated_ = pose_edges_.size()-kMaxPoseEdges;
        // Fix first activated pose, to reduce degrees of freedom
        g2o::VertexSE2* v0 = static_cast<g2o::VertexSE2*>(pose_edges_[pose_edges_deactivated_]->vertex(0));
        v0->setFixed(true);
    }
    if(landmark_edges_.size()-landmark_edges_deactivated_ > kMaxLandmarkEdges){
        for (int i=landmark_edges_deactivated_; i<landmark_edges_.size()-kMaxLandmarkEdges; i++){
            landmark_edges_[i]->setLevel(2); // Deactivate edge
        }
        landmark_edges_deactivated_ = landmark_edges_.size()-kMaxLandmarkEdges;
    }
}


void GraphSlam::fix_map(){ 
    for (auto vertex : optimizer_.vertices()) {
        g2o::VertexPointXY* landmark_vertex = dynamic_cast<g2o::VertexPointXY*>(vertex.second);
        if (landmark_vertex != nullptr) {
            landmark_vertex->setFixed(true);
        }
    }
    map_fixed_ = true;
}

void GraphSlam::publish_map(){
    pcl::PointCloud<ConeXYZColorScore> map;
    for (Landmark* landmark : DA.map_){
        if (landmark->disabled_ || landmark->num_observations_ <= 3){
            continue;
        }
        if(landmark->color_==UNCOLORED){
            Eigen::Vector2d local_pos = global_to_local(landmark->world_position_);
            if(local_pos.norm() < 6 && local_pos[0] < 0){

                if(local_pos.y() > 0.5){
                    landmark->color_ = BLUE;
                } else if (local_pos.y() < -0.5){
                    landmark->color_ = YELLOW;
                }
            }
        }
        ConeXYZColorScore cone;
        cone.x = landmark->world_position_(0);
        cone.y = landmark->world_position_(1);
        cone.z = 0;
        cone.color = landmark->color_;
        cone.score = 1.0;
        map.push_back(cone);
    }
    
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(map, map_msg);
    map_msg.header.frame_id = "arussim/world";
    map_pub_->publish(map_msg);
    if(map_fixed_){
        final_map_pub_->publish(map_msg);
    }
}


void GraphSlam::check_finish_line(){
    bool lap_finished = vehicle_pose_(0) > kFinishLineOffset-1
                        && vehicle_pose_(0) < kFinishLineOffset+1
                        && vehicle_pose_(1) > -kTrackWidth/2
                        && vehicle_pose_(1) < kTrackWidth/2
                        && driven_distance_ > kMinLapDistance;
    if (lap_finished){
        driven_distance_ = 0.0;
        lap_count_++;

        std_msgs::msg::Int16 lap_count_msg;
        lap_count_msg.data = lap_count_;
        lap_count_pub_->publish(lap_count_msg);
    }

    if (lap_count_ == 1){
        fix_map();
    }
}


/////////////////////////////////////////////////////////////////////////
/////////////////////////////// UTILS ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////


// Get the global position given the local position relative to the actual vehicle pose
Eigen::Vector2d GraphSlam::local_to_global(const Eigen::Vector2d& local_pos) {
    const double phi = vehicle_pose_(2);
    const double sinphi = std::sin(phi);
    const double cosphi = std::cos(phi);
    const Eigen::Matrix2d R = (Eigen::Matrix2d() << cosphi, -sinphi, sinphi, cosphi).finished();

    return R * local_pos + vehicle_pose_.head(2);
}


// Get the local position relative to the actual vehicle pose given the global position
Eigen::Vector2d GraphSlam::global_to_local(const Eigen::Vector2d& global_pos) {
    const double phi = -vehicle_pose_(2);
    const double sinphi = std::sin(phi);
    const double cosphi = std::cos(phi);
    const Eigen::Matrix2d R = (Eigen::Matrix2d() << cosphi, -sinphi, sinphi, cosphi).finished();

    return R * (global_pos - vehicle_pose_.head(2));
}

void GraphSlam::send_tf() {
	geometry_msgs::msg::TransformStamped transformSt;
	transformSt.header.stamp = this->now();
	transformSt.header.frame_id = "arussim/world";
	transformSt.child_frame_id = "slam/vehicle";
	tf2::Quaternion q;
	transformSt.transform.translation.x = vehicle_pose_(0);
	transformSt.transform.translation.y = vehicle_pose_(1);
	q.setRPY(0, 0, vehicle_pose_(2));
	transformSt.transform.rotation.x = q.x();
	transformSt.transform.rotation.y = q.y();
	transformSt.transform.rotation.z = q.z();
	transformSt.transform.rotation.w = q.w();

	tf_broadcaster_->sendTransform(transformSt);
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphSlam>();

    // Create a MultiThreadedExecutor to allow parallel processing 
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
}
