#include "graph_slam/graph_slam_node.hpp"
// #include "data_association_euclidean.cpp"

GraphSlam::GraphSlam() : Node("graph_slam")
{ 

    this->declare_parameter("finish_line_offset", 0.0);
    this->declare_parameter("track_width", 3.0);
    this->declare_parameter("min_lap_distance", 30.0);
    this->declare_parameter("write_csv", false);
    this->get_parameter("finish_line_offset", kFinishLineOffset);
    this->get_parameter("track_width", kTrackWidth);
    this->get_parameter("min_lap_distance", kMinLapDistance);
    this->get_parameter("write_csv", kWriteCSV);

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
    pose_vertices_.push_back(init_vertex);
    optimizer_.addVertex(init_vertex);
    // TODO: fix first vertex
    
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
    lap_count_pub_ = this->create_publisher<std_msgs::msg::Int16>("/slam/lap_count", 10);

    state_sub_ = this->create_subscription<common_msgs::msg::State>("/car_state/state", 10, 
                    std::bind(&GraphSlam::state_callback, this, std::placeholders::_1), collector_options);
    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/arussim/perception", 10,
                    std::bind(&GraphSlam::perception_callback, this, std::placeholders::_1), collector_options);           
    // TODO: run optimizer depending on the number of edges/itereations
    optimizer_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                    std::bind(&GraphSlam::optimizer_callback, this), optimizer_callback_group_);
}


void GraphSlam::state_callback(const common_msgs::msg::State::SharedPtr msg)
{   
    // Update pose estimation from odometry
    double phi = vehicle_pose_(2);
    B.block(0, 0, 2, 2) << std::cos(phi), -std::sin(phi), std::sin(phi), std::cos(phi);
    u << msg->vx, msg->vy, msg->r;
    vehicle_pose_ += B * u * dt; // TODO check phi in range [-pi, pi]
    driven_distance_ += (B * u * dt).norm();
    
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
    vertices_to_add_.push_back(pose_vertex);
    

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

        observed_landmarks.push_back(Landmark(Eigen::Vector2d(cone.x, cone.y), vehicle_pose_));
    }
    
    DA.match_observations(observed_landmarks, unmatched_landmarks);
    for (auto landmark : unmatched_landmarks) {
        landmark.id_ = 2*DA.map_.size(); // Use even ids for landmark vertices
        Landmark* new_landmark = new Landmark(landmark); // Copy the landmark to avoid memory issues
        DA.map_.push_back(new_landmark);

        // Add a new landmark vertex 
        g2o::VertexPointXY* landmark_vertex = new g2o::VertexPointXY();
        landmark_vertex->setId(landmark.id_);
        landmark_vertex->setEstimate(landmark.world_position_);
        landmark_vertices_.push_back(landmark_vertex);
        vertices_to_add_.push_back(landmark_vertex);

        // Add an edge between the last pose vertex and the new landmark vertex 
        g2o::EdgeSE2PointXY* edge = new g2o::EdgeSE2PointXY();
        edge->vertices()[0] = last_pose_vertex;
        edge->vertices()[1] = landmark_vertex;
        edge->setMeasurement(landmark.local_position_);
        edge->setInformation(R.inverse());
        landmark_edges_.push_back(edge);
        edges_to_add_.push_back(edge);
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
        landmark_edges_.push_back(edge);
        edges_to_add_.push_back(edge);
    }

    // std::cout << "Number of vertices: " << optimizer_.vertices().size() << std::endl;
    // std::cout << "Number of edges: " << optimizer_.edges().size() << std::endl;

    publish_map();
}

void GraphSlam::optimizer_callback(){
    addVerticesAndEdges();
    optimizer_.initializeOptimization();
    optimizer_.optimize(200);
    update_data_association_map();
    if(kWriteCSV){
        write_csv_log();
    }
}

// Update the world position of the landmarks in the data association map after optimization
// Landmarks must be pointers to be updated
void GraphSlam::update_data_association_map(){
    for (Landmark* landmark : DA.map_){
        for (auto vertex : optimizer_.vertices()) {
            g2o::VertexPointXY* landmark_vertex = dynamic_cast<g2o::VertexPointXY*>(vertex.second);
            if (landmark_vertex != nullptr && landmark_vertex->id() == landmark->id_) {
                landmark->world_position_ = landmark_vertex->estimate();
            }
        }
    }
}

void GraphSlam::addVerticesAndEdges(){
    for (auto vertex: vertices_to_add_){
        optimizer_.addVertex(vertex);
    }
    vertices_to_add_.clear();
    for (auto edge: edges_to_add_){
        optimizer_.addEdge(edge);
    }
    edges_to_add_.clear();
}

void GraphSlam::publish_map(){
    pcl::PointCloud<ConeXYZColorScore> map;
    for (Landmark* landmark : DA.map_){

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
}

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


void GraphSlam::write_csv_log(){
    
    const char* homeDir = getenv("HOME"); 
    if (!homeDir) {
        return;
    }

    std::filesystem::path filePath = std::filesystem::path(homeDir) / "ARUS_logs/slam/graph_slam_log.csv";

    std::ofstream graph_csv(filePath);

    if (!graph_csv.is_open()) {
        std::cerr << "Couldn't open file for writing." << std::endl;
        return;
    }

    for (auto vertex : optimizer_.vertices()) {
        g2o::VertexSE2* pose_vertex = dynamic_cast<g2o::VertexSE2*>(vertex.second);
        if (pose_vertex != nullptr) {
            graph_csv << "pose_vertex," << pose_vertex->id() << "," 
                                << pose_vertex->estimate().translation().x() << "," 
                                << pose_vertex->estimate().translation().y() << "," 
                                << pose_vertex->estimate().rotation().angle() << "\n";
        }
        g2o::VertexPointXY* landmark_vertex = dynamic_cast<g2o::VertexPointXY*>(vertex.second);
        if (landmark_vertex != nullptr) {
            graph_csv << "landmark_vertex," << landmark_vertex->id() << "," 
                                << landmark_vertex->estimate().x() << "," 
                                << landmark_vertex->estimate().y() << "\n";
        }
    }

    for(auto edge : optimizer_.edges()){
        g2o::EdgeSE2PointXY* landmark_edge = dynamic_cast<g2o::EdgeSE2PointXY*>(edge);
        if (landmark_edge != nullptr) {
            auto* pose_vertex = dynamic_cast<g2o::VertexSE2*>(landmark_edge->vertices()[0]);
            if (pose_vertex) {
                graph_csv << "landmark_edge," << pose_vertex->estimate().translation().x() << "," 
                                << pose_vertex->estimate().translation().y() << ",";
            }
            auto* landmark_vertex = dynamic_cast<g2o::VertexPointXY*>(landmark_edge->vertices()[1]);
            if (landmark_vertex) {
                graph_csv << landmark_vertex->estimate().x() << "," 
                             << landmark_vertex->estimate().y() << "\n";
            }
        }
    }
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