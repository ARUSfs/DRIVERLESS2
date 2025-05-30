/**
 * @file path_planning_node.cpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @brief Main file for the Path Planning node. Contains the main function and the implementation
 * of the methods to achieve a robust and reliable path planning algorithm for the ARUS Team
 * which extracts the midpoints of the track that the ART will follow.
 */
#include "path_planning/path_planning_node.hpp"

PathPlanning::PathPlanning() : Node("path_planning")
{
    // Declare and get parameters
    // Debug
    this->declare_parameter<bool>("debug", true);

    this->get_parameter("debug", kDebug);

    // Topics
    this->declare_parameter<std::string>("map_topic", "/slam/map");
    this->declare_parameter<std::string>("lap_count_topic", "/slam/lap_count");
    this->declare_parameter<std::string>("car_info_topic", "/car_state/car_info");
    this->declare_parameter<std::string>("optimizer_topic", "/trajectory_optimization/trajectory");
    this->declare_parameter<std::string>("triangulation_topic", "/path_planning/triangulation");
    this->declare_parameter<std::string>("trajectory_topic", "/path_planning/trajectory");
    this->declare_parameter<std::string>("track_limits_topic", "/path_planning/track_limits");

    this->get_parameter("map_topic", kMapTopic);
    this->get_parameter("lap_count_topic", kLapCountTopic);
    this->get_parameter("car_info_topic", kCarInfoTopic);
    this->get_parameter("optimizer_topic", kOptimizerTopic);
    this->get_parameter("triangulation_topic", kTriangulationTopic);
    this->get_parameter("trajectory_topic", kTrajectoryTopic);
    this->get_parameter("track_limits_topic", kTrackLimitsTopic);

    // Triangulation
    this->declare_parameter<double>("max_tri_len", 7);
    
    this->get_parameter("max_tri_len", kMaxTriLen);

    // Route
    this->declare_parameter<double>("angle_coeff", 1.0);
    this->declare_parameter<double>("len_coeff", 1.0);
    this->declare_parameter<double>("smooth_coeff", 1.2);
    this->declare_parameter<double>("prev_route_bias", 0.75);
    this->declare_parameter<int>("route_back", 10);
    this->declare_parameter<bool>("use_buffer", false);
    this->declare_parameter<double>("time_to_close", 1.0);
    this->declare_parameter<bool>("stop_after_closing", false);

    this->get_parameter("angle_coeff", kAngleCoeff);
    this->get_parameter("len_coeff", kLenCoeff);
    this->get_parameter("smooth_coeff", kSmoothCoeff);
    this->get_parameter("prev_route_bias", kPrevRouteBias);
    this->get_parameter("route_back", kRouteBack);
    this->get_parameter("use_buffer", kUseBuffer);
    this->get_parameter("time_to_close", kTimeToClose);
    this->get_parameter("stop_after_closing", kStopAfterClosing);
    
    // Profile creation
    this->declare_parameter<double>("v_max", 10.0);
    this->declare_parameter<double>("ay_max", 5.0);
    this->declare_parameter<double>("ax_max", 5.0);

    this->get_parameter("v_max", kMaxVel);
    this->get_parameter("ay_max", kMaxYAcc);
    this->get_parameter("ax_max", kMaxXAcc);

    // Subscribers
    map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kMapTopic, 10, std::bind(&PathPlanning::map_callback, this, std::placeholders::_1));
    lap_count_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        kLapCountTopic, 10, std::bind(&PathPlanning::lap_count_callback, this, std::placeholders::_1));
    car_info_sub_ = this->create_subscription<common_msgs::msg::CarInfo>(
        kCarInfoTopic, 10, std::bind(&PathPlanning::car_info_callback, this, std::placeholders::_1));
    optimizer_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kOptimizerTopic, 10, std::bind(&PathPlanning::optimizer_callback, this, std::placeholders::_1));
    
    // Publishers
    triangulation_pub_ = this->create_publisher<common_msgs::msg::Triangulation>(kTriangulationTopic, 10);
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>(kTrajectoryTopic, 10);
    track_limits_pub_ = this->create_publisher<common_msgs::msg::TrackLimits>(kTrackLimitsTopic, 10);

}

void PathPlanning::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr per_msg)
{
    double start = this->now().seconds();
    // Get the track limits in the second lap
    if (lap_count_ > 0 && x_>3 && !track_limits_sent_){
        common_msgs::msg::TrackLimits track_limits_msg = this->create_track_limits_msg(TL_triang_, TL_tri_indices_);
        if (track_limits_msg.right_limit.size()>0) track_limits_pub_->publish(track_limits_msg);
        track_limits_sent_ = true;
    }


    // Publish the closing route if selected in config file and shutdown the node
    if (closing_route_.size() > 0){
        common_msgs::msg::Trajectory trajectory_msg = this->create_trajectory_msg(closing_route_);
        if (trajectory_msg.points.size()>0) trajectory_pub_->publish(trajectory_msg);
        
        if (kStopAfterClosing){   
            common_msgs::msg::TrackLimits track_limits_msg = this->create_track_limits_msg(TL_triang_, TL_tri_indices_);
            if (track_limits_msg.right_limit.size()>0) track_limits_pub_->publish(track_limits_msg);
            rclcpp::shutdown();
        }
        return;
    }

    // Save the point cloud as a pcl object from ROS2 msg
    pcl::fromROSMsg(*per_msg, pcl_cloud_);

    // Check if the point cloud is empty and return if it is
    if(pcl_cloud_.size() < 3){
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Empty point cloud");
        return;
    }


    std::vector<ConeXYZColorScore> back_edge = this->get_back_edge();
    if (back_route_.size() < 2 && back_edge.size() != 2){
        pcl_cloud_.push_back(ConeXYZColorScore(0, -1.5, 0, YELLOW, -1));
        pcl_cloud_.push_back(ConeXYZColorScore(0, 1.5, 0, BLUE, -1));
        back_edge.clear();
        back_edge.push_back(ConeXYZColorScore(0, -1.5, 0, YELLOW, -1));
        back_edge.push_back(ConeXYZColorScore(0, 1.5, 0, BLUE, -1));
    }

    if (back_edge.size()!=2) {
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Back edge not found");
        return;
    }

    // Add the back edge to the track limits
    this->add_to_track_limits(back_edge);

    // Create the triangulation
    CDT::Triangulation<double> triang;
    triang = this->create_triangulation(pcl_cloud_);

    // Get attributes from the triangulation
    vertices_ = triang.vertices;
    triangles_ = triang.triangles;

    // Publish the triangulation as a ROS2 message
    common_msgs::msg::Triangulation triangulation_msg;
    if (triang.isFinalized()){
        triangulation_msg = this->create_triangulation_msg(triang);
    }
    triangulation_pub_ -> publish(triangulation_msg);
    
    if (kDebug) {
        double triangulation_time = this->now().seconds() - start;
        RCLCPP_INFO(this->get_logger(), "Triangulation time: %f", triangulation_time);
    }
    
    // Get the nearest triangle containing the back edge
    int nearest_tri = -1;
    double min_distance = INFINITY;
    int edge_v0_ind = get_vertex_index(CDT::V2d<double>::make(back_edge[0].x,back_edge[0].y), triang);
    int edge_v1_ind = get_vertex_index(CDT::V2d<double>::make(back_edge[1].x,back_edge[1].y), triang);
    if (edge_v0_ind == -1 || edge_v1_ind == -1){
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Back edge vertices not found");
        return;
    }
    for (int i : get_triangles_from_vert(edge_v0_ind, triang)){
        if (in(i, get_triangles_from_vert(edge_v1_ind, triang))){
            CDT::V2d<double> centroid = compute_centroid(i, triang.triangles, triang.vertices);
            double d = (centroid.x-x_) * (centroid.x-x_) + (centroid.y-y_) * (centroid.y-y_);
            if (d < min_distance){
                min_distance = d;
                nearest_tri = i;
            }
        }
    }

    if (nearest_tri == -1){
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Nearest triangle not found");
        return;
    }

    // Get passed vertices from the back route
    std::vector<int> passed_vertices;
    for (auto p : back_points_){
        int ind_1 = get_vertex_index(CDT::V2d<double>::make(p.x, p.y), triang, 0.95);
        if (ind_1 == -1) continue;
        if (in(ind_1, passed_vertices)) continue;
        passed_vertices.push_back(ind_1);
    }

    SimplexTree tree;
    tree = SimplexTree(triangles_, nearest_tri, {edge_v0_ind, edge_v1_ind}, pcl_cloud_, passed_vertices,
                       yaw_, kAngleCoeff, kLenCoeff);
    auto c0 = tree.best_route_[0];
    auto c2 = tree.best_route_[2];
    double angle_diff = std::abs(atan2(c2.y-c0.y, c2.x-c0.x)-yaw_);
    angle_diff = std::min(angle_diff, 2*M_PI-angle_diff);
    if (angle_diff < M_PI/2){
        best_midpoint_route_ = tree.best_route_;
    } else {
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Best route looking backwards");
        return;
    }

    if (kDebug) {
        double tree_time = this->now().seconds() - start;
        RCLCPP_INFO(this->get_logger(), "Simplex tree time: %f", tree_time);
    }
    
    // Get the final route using the buffer if selected in config file
    std::vector<ConeXYZColorScore> final_route;

    if (kUseBuffer){
        previous_midpoint_routes_.push_back(best_midpoint_route_);
        final_route = this->get_final_route();
    } else {
        final_route = best_midpoint_route_;
    }

    if (final_route.size() <3) {
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Final route is empty");
        return;
    }

    ConeXYZColorScore back_point;
    bool route_closed = false;
    bool append_point = true;
    if (back_edge.size() == 2) {
        back_point = ConeXYZColorScore((back_edge[0].x+back_edge[1].x)/2,
                                       (back_edge[0].y+back_edge[1].y)/2, 0, UNCOLORED, 1);

        for (auto p: back_route_){
            if (distance(back_point, p) < 0.5){
                append_point = false;
                break;
            }
        }

        if (append_point) {
            back_route_.push_back(back_point);
            back_points_.push_back(back_edge[0]);
            back_points_.push_back(back_edge[1]);
        } else if (back_route_.size() > 10 && distance(back_point, back_route_[0]) < 3.0){
            route_closed = true;
        }
    }

    std::vector<ConeXYZColorScore> full_route = back_route_;
    double prev_angle = 0.0;
    if (back_route_.size()>=2) prev_angle = atan2(back_route_.back().y-back_route_[back_route_.size()-2].y,
                                                  back_route_.back().x-back_route_[back_route_.size()-2].x);
    for (int i=0 ; i<final_route.size(); i++){
        auto c = final_route[i];
        append_point = true;
        double angle_diff = std::abs(atan2(c.y-full_route.back().y, c.x-full_route.back().x)-prev_angle);
        angle_diff = std::min(angle_diff, 2*M_PI-angle_diff);

        if ( distance(c, ConeXYZColorScore()) > 3.0 || back_route_.size()<10){

            // Check if the point is too close to the last 10 points in the route
            for (int j=1; j<=10; j++){
                if (distance(c, full_route[full_route.size()-j]) < 0.5){
                    append_point = false;
                    break;
                }
            }

            if (append_point && angle_diff < M_PI/2){ // Ensure no duplicates
                prev_angle = atan2(c.y-full_route.back().y, c.x-full_route.back().x);
                full_route.push_back(c);
            } 
        } else {
            if (full_route.size() > 0.5*pcl_cloud_.size()){
                route_closed = true;
            }
            full_route.erase(full_route.begin());
            full_route.push_back(full_route[0]);
            break;
        }
    }

    if (tree.ending_routes_.size()>0 && route_closed) {
        TL_triang_ = triangles_;
        TL_tri_indices_ = tree.best_index_route_;
    }

    // Use closing route if route is closed
    if (route_closed || lap_count_ > 0) {
        if ((this->now() - last_unfinished_).seconds() > kTimeToClose){
            closing_route_ = full_route;
        }
    } else {
        last_unfinished_ = this -> now();
    }

    // Publish the best trajectory
    trajectory_pub_ -> publish(this->create_trajectory_msg(full_route));
    
    if (kDebug) {
        double trajectory_time = this->now().seconds() - start;
        RCLCPP_INFO(this->get_logger(), "Trajectory time: %f", trajectory_time);
    }
}

void PathPlanning::car_info_callback(const common_msgs::msg::CarInfo::SharedPtr state_msg)
{
    x_ = state_msg->x;
    y_ = state_msg->y;
    yaw_ = state_msg->yaw;
    target_speed_ = state_msg->target_speed;
    vx_ = state_msg->vx;
    vy_ = state_msg->vy;
    v_ = hypot(vx_, vy_);
    origin_ = ConeXYZColorScore(x_, y_, 0, UNCOLORED, -1);
}

void PathPlanning::lap_count_callback(const std_msgs::msg::Int16::SharedPtr lap_msg)
{
    lap_count_ = lap_msg->data;
}

void PathPlanning::optimizer_callback(const common_msgs::msg::Trajectory::SharedPtr optimizer_msg)
{   
    if (track_limits_sent_) rclcpp::shutdown(); // End only if the track limits have been sent
}

CDT::Triangulation<double> PathPlanning::create_triangulation(pcl::PointCloud<ConeXYZColorScore> input_cloud){
    // Initialize empty triangulation and empty points vector
    CDT::Triangulation<double> triangulation;
    std::vector<CDT::V2d<double>> points;
    // Add the points to the triangulation from the cloud in CDT format
    for (std::size_t i = 0; i<input_cloud.size();i++){
        ConeXYZColorScore cone = input_cloud.points[i];
        CDT::V2d<double> point = CDT::V2d<double>::make(cone.x, cone.y);
        points.push_back(point);
    }
    triangulation.insertVertices(points);
    // Delete the super triangle to avoid errors in next steps
    triangulation.eraseSuperTriangle();
    // Iterate over the triangles and delete the ones that don't meet the conditions
    CDT::TriangleVec triangles = triangulation.triangles;
    CDT::Triangulation<double>::V2dVec vertices = triangulation.vertices;
    CDT::TriIndUSet deleted_tri;
    for (int i=0; i<triangles.size(); i++) {
        // Angle and length calculations
        CDT::VerticesArr3 vert_ind = triangles[i].vertices;
        ConeXYZColorScore a = input_cloud.points[vert_ind[0]];
        ConeXYZColorScore b = input_cloud.points[vert_ind[1]];
        ConeXYZColorScore c = input_cloud.points[vert_ind[2]];
        CDT::V2d<double> ab, bc, ca;
        double a_angle, b_angle, c_angle;
        ab = CDT::V2d<double>::make(b.x-a.x, b.y-a.y);
        bc = CDT::V2d<double>::make(c.x-b.x, c.y-b.y);
        ca = CDT::V2d<double>::make(a.x-c.x, a.y-c.y);

        // Delete triangles with long edges and all vertices with the same color
        if (lap_count_==0 && ((a.x == origin_.x and a.y == origin_.y) or     // Check if the origin vertex is
                              (b.x == origin_.x and b.y == origin_.y) or     // in the triangle and skip it
                              (c.x == origin_.x and c.y == origin_.y))){     // (origin vertex is the car position)
            continue;
        }

        if (a.color != UNCOLORED and a.color == b.color and b.color == c.color){
            deleted_tri.insert(i);
        } else if (distance(a,b) > kMaxTriLen or distance(b,c) > kMaxTriLen or distance(c,a) > kMaxTriLen){
            deleted_tri.insert(i);
        }
    }
    triangulation.removeTriangles(deleted_tri);
    return triangulation;
}

std::vector<ConeXYZColorScore> PathPlanning::get_back_edge(){

    std::vector<ConeXYZColorScore> back_edge = {}; 

    pcl::PointCloud<ConeXYZColorScore> local_cloud = pcl_cloud_;
    local_cloud.points.push_back(origin_);
    
    // Create the triangulation
    CDT::Triangulation<double> triang = this->create_triangulation(local_cloud);

    // Initializing it from each of the origin triangles
    int orig_index = get_vertex_index(CDT::V2d<double>::make(x_,y_), triang);
    if (orig_index == -1){
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Origin vertex not found");
        return back_edge;
    }
    std::vector<int> o_triangles = get_triangles_from_vert(orig_index, triang);        
        
    for (int i = 0; i<o_triangles.size(); i++){
        // Calculate the centroid of the triangle and the angle difference with the car yaw
        CDT::V2d<double> centroid = compute_centroid(o_triangles[i], triang.triangles, triang.vertices);
        double angle_diff = abs(atan2(centroid.y-y_, centroid.x-x_)-yaw_);
        double corrected_angle_diff = std::min(angle_diff, 2*M_PI-angle_diff);

        // Back edge is behind the car (angle diff > pi/2)
        if (corrected_angle_diff >  M_PI/2) {
            for (int j = 0; j<3; j++){
                if (back_edge.size()==2 || triang.triangles[o_triangles[i]].vertices[j] == orig_index){
                    continue;
                }
                if (pcl_cloud_[triang.triangles[o_triangles[i]].vertices[j]].color != UNCOLORED){
                    back_edge.push_back(pcl_cloud_[triang.triangles[o_triangles[i]].vertices[j]]);
                }
            }
            if (back_edge.size() != 2) back_edge.clear();
            if (back_edge.size() == 2 && back_edge[0].color == back_edge[1].color){
                back_edge.clear();
            }
        }
    }

    return back_edge;
}

std::vector<ConeXYZColorScore> PathPlanning::get_final_route(){
    // Create candidate to final route from cost calculations
    std::vector<ConeXYZColorScore> final_route = previous_midpoint_routes_.back();

    // Return the final route if there are not enough previous routes
    if (previous_midpoint_routes_.size() < kRouteBack+1){
        return final_route;
    }

    // Create a list of the last routes
    std::vector<std::vector<ConeXYZColorScore>> last_routes(previous_midpoint_routes_.end()-kRouteBack-1,
                                                           previous_midpoint_routes_.end()-1);

    // Count the number of points in the last routes that are in the final route
    int route_count = 0;
    int total_points = 0;
    for (int i = 0; i<kRouteBack; i++){
        int point_count = compare_lists(last_routes[i], final_route, 0.25);
        route_count += point_count;
        total_points += last_routes[i].size();
    }

    // Create threshold to validate last route based on previous route lengths
    double threshold = total_points*kPrevRouteBias;
    // If the route is validated, return it and add to previous routes.
    if (route_count > threshold){
        invalid_counter_ = 0;
        return final_route;
    } else {   // Otherwise, return the previous route
        invalid_counter_++;
        return previous_midpoint_routes_[previous_midpoint_routes_.size() - 1 - invalid_counter_];
    }
}

common_msgs::msg::Trajectory PathPlanning::create_trajectory_msg(
            std::vector<ConeXYZColorScore> route, bool smooth){
    int route_size = route.size();
    int degree = 2;
    common_msgs::msg::Trajectory trajectory_msg;
    double acum = 0.0;

    std::vector<double> xp, yp, xpp, ypp, v_grip, s, k, speed_profile, acc_profile;

    if (route_size <= 3){
        return trajectory_msg;
    } else if (!smooth){
        for (int i = 0; i<route_size; i++){
            common_msgs::msg::PointXY point;
            point.x = route[i].x;
            point.y = route[i].y;
            trajectory_msg.points.push_back(point);
        }
        return trajectory_msg;
    }

    // Avoid division by zero
    for (auto p : route){
        if (std::abs(p.x)<0.001) p.x = 0.001;
        if (std::abs(p.y)<0.001) p.y = 0.001;
    }

    // Perform laplacian smoothing previous to spline fitting
    std::vector<double> new_x, new_y;
    new_x.push_back(route[0].x);
    new_y.push_back(route[0].y);
    for (int i = 1; i<route_size-1;i++){
        new_x.push_back(((3-kSmoothCoeff)/2*route[i-1].x+kSmoothCoeff*route[i].x+(3-kSmoothCoeff)/2*route[i+1].x)/3);
        new_y.push_back(((3-kSmoothCoeff)/2*route[i-1].y+kSmoothCoeff*route[i].y+(3-kSmoothCoeff)/2*route[i+1].y)/3);
    }
    new_x.push_back(route[route_size-1].x);
    new_y.push_back(route[route_size-1].y);

    Eigen::MatrixXd control_points(2, route_size);
    for (int i = 0; i<route_size; i++){
        control_points(0, i) = new_x[i];
        control_points(1, i) = new_y[i];
    }

    Eigen::Spline<double, 2> b_spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(
        control_points, degree);

    // Add the points to the trajectory message
    for (double i = 0; i<route_size*10-1; i++){
        Eigen::Vector2d point_1 = b_spline(i/(10*route_size));
        Eigen::Vector2d point_2 = b_spline((i+1)/(10*route_size));
        double x = point_1(0);
        double y = point_1(1);
        double dx = point_2(0)-x;
        double dy = point_2(1)-y;
        double dist = hypot(dx, dy);
        common_msgs::msg::PointXY point;
        point.x = x;
        point.y = y;
        trajectory_msg.points.push_back(point);
        xp.push_back(dx);
        yp.push_back(dy);
        acum += dist;
        s.push_back(acum);
        speed_profile.push_back(0.0);
    }

    xp.push_back(xp.back());
    yp.push_back(yp.back());

    for (int i = 0; i<xp.size()-1; i++){
        xpp.push_back(xp[i+1]-xp[i]);
        ypp.push_back(yp[i+1]-yp[i]);
    }
    xpp.push_back(xpp.back());
    ypp.push_back(ypp.back());

    for (int i = 0; i<xpp.size(); i++){
        double den = pow(pow(xp[i],2)+pow(yp[i],2), 1.5);
        double val;
        if (std::abs(den) < 0.001){
            val = 0;
        } else{
            val = (xp[i]*ypp[i]-yp[i]*xpp[i])/den;
        }
        k.push_back(val);
    }

    for (const auto &val : k){
        double v = std::min(kMaxVel, sqrt(kMaxYAcc/std::max(abs(val), 0.001)));
        v_grip.push_back(v);
    }

    speed_profile[0] = std::max(v_, 0.5);

    for (int i = 1; i<speed_profile.size(); i++){
        double ds = s[i]-s[i-1];
        speed_profile[i] = sqrt(pow(speed_profile[i-1], 2) + 2*kMaxXAcc*ds);
        if (speed_profile[i] > v_grip[i]){
            speed_profile[i] = v_grip[i];
        }
    }

    for (int i = speed_profile.size()-2; i>=0; i--){
        double ds = s[i+1]-s[i];
        double v_brake = sqrt(pow(speed_profile[i+1], 2) + 2*kMaxXAcc*ds);
        if (v_brake < speed_profile[i]){
            speed_profile[i] = v_brake;
        }
    }

    acc_profile.push_back(0.0);
    for (int i = 0; i<speed_profile.size()-1; i++){
        double ds = s[i+1]-s[i];
        if (std::abs(ds) > 0.001){
            acc_profile.push_back((pow(speed_profile[i+1], 2)-pow(speed_profile[i], 2))/(2*ds));
        } else {
            acc_profile.push_back(0.0);
        }
    }

    for (int i = 0; i<s.size(); i++){
        if (std::isnan(speed_profile[i])) speed_profile[i] = 0.0;
        if (std::isnan(acc_profile[i])) acc_profile[i] = 0.0;

        trajectory_msg.k.push_back(k[i]);
        trajectory_msg.s.push_back(s[i]);
        trajectory_msg.speed_profile.push_back(speed_profile[i]);
        trajectory_msg.acc_profile.push_back(acc_profile[i]);
    }

    return trajectory_msg;
}

void PathPlanning::add_to_track_limits(std::vector<ConeXYZColorScore> back_edge){
    if (back_edge.size() != 2) return;
    if (back_edge[0].score == -1 || back_edge[1].score == -1) return;
    for (int i = 0; i < 2; i++){
        if (back_edge[i].color == UNCOLORED) continue;
        switch (back_edge[i].color) {
            case YELLOW:
                if (right_limit_.size() > 0){
                    if (distance(back_edge[i], right_limit_.back()) < 0.5){
                        continue;
                    } else {
                        right_limit_.push_back(back_edge[i]);
                    }
                } else {
                    right_limit_.push_back(back_edge[i]);
                }
                break;
            case BLUE:
                if (left_limit_.size() > 0){
                    if (distance(back_edge[i], left_limit_.back()) < 0.5){
                        continue;
                    } else {
                        left_limit_.push_back(back_edge[i]);
                    }
                } else {
                    left_limit_.push_back(back_edge[i]);
                }
                break;
            default:
                break;
        }
    }
}

common_msgs::msg::TrackLimits PathPlanning::create_track_limits_msg(CDT::TriangleVec triang, 
                                                                std::vector<int> triangles_route){
    CDT::Triangle last_triangle = triang[triangles_route.back()];
    CDT::NeighborsArr3 neighbors = last_triangle.neighbors;
    for (int i = 0; i<3; i++){
        if (neighbors[i] < triang.size() && !in(neighbors[i], triangles_route)){
            triangles_route.push_back(neighbors[i]);
        }
    }
    std::vector<int> left_limit, right_limit;
    for (int i = 0; i<triangles_route.size(); i++){
        CDT::Triangle triangle = triang[triangles_route[i]];
        CDT::VerticesArr3 vertices = triangle.vertices;
        for (int j = 0; j<3; j++){
            ConeXYZColorScore cone = pcl_cloud_.points[vertices[j]];
            if (cone.score == -1){
                continue;
            }
            int vertex = vertices[j];
            if (in(vertex, left_limit) || in(vertex, right_limit)){
                continue;
            }
            switch (cone.color){
            case BLUE:
                left_limit.push_back(vertex);
                break;
            case YELLOW:
                right_limit.push_back(vertex);
                break;
            case UNCOLORED:
                if (i < triangles_route.size()-1) {
                    std::vector<double> yaw_vector, cone_vector;
                    CDT::V2d<double> centroid_1 = compute_centroid(triangles_route[i], triang, vertices_);
                    CDT::V2d<double> centroid_2 = compute_centroid(triangles_route[i+1], triang, vertices_);
                    double route_angle = atan2(centroid_2.y-centroid_1.y, centroid_2.x-centroid_1.x);
                    yaw_vector = {cos(route_angle), sin(route_angle)};
                    cone_vector = {cone.x-centroid_1.x, cone.y-centroid_1.y};
                    bool is_positive_angle = yaw_vector[0]*cone_vector[1]-yaw_vector[1]*cone_vector[0] > 0;
                    if (is_positive_angle){
                        left_limit.push_back(vertex);
                    } else {
                        right_limit.push_back(vertex);
                    }
                } else {
                    std::vector<double> yaw_vector, cone_vector;
                    CDT::V2d<double> centroid_1 = compute_centroid(triangles_route[i], triang, vertices_);
                    CDT::V2d<double> centroid_2 = CDT::V2d<double>::make(1.5, 0);
                    double route_angle = atan2(centroid_2.y-centroid_1.y, centroid_2.x-centroid_1.x);
                    yaw_vector = {cos(route_angle), sin(route_angle)};
                    cone_vector = {cone.x-centroid_1.x, cone.y-centroid_1.y};
                    bool is_positive_angle = yaw_vector[0]*cone_vector[1]-yaw_vector[1]*cone_vector[0] > 0;
                    if (is_positive_angle){
                        left_limit.push_back(vertex);
                    } else {
                        right_limit.push_back(vertex);
                    }
                }
                break;
            }
        }
    }
    common_msgs::msg::TrackLimits track_limits_msg;
    for (int i = 0; i<left_limit_.size(); i++){
        common_msgs::msg::PointXY point;
        point.x = left_limit_[i].x;
        point.y = left_limit_[i].y;
        track_limits_msg.left_limit.push_back(point);
    }
    for (int i = 0; i<left_limit.size(); i++){
        common_msgs::msg::PointXY point;
        point.x = pcl_cloud_.points[left_limit[i]].x;
        point.y = pcl_cloud_.points[left_limit[i]].y;
        track_limits_msg.left_limit.push_back(point);
    }
    for (int i = 0; i<right_limit_.size(); i++){
        common_msgs::msg::PointXY point;
        point.x = right_limit_[i].x;
        point.y = right_limit_[i].y;
        track_limits_msg.right_limit.push_back(point);
    }
    for (int i = 0; i<right_limit.size(); i++){
        common_msgs::msg::PointXY point;
        point.x = pcl_cloud_.points[right_limit[i]].x;
        point.y = pcl_cloud_.points[right_limit[i]].y;
        track_limits_msg.right_limit.push_back(point);
    }
    track_limits_msg.trajectory = this->create_trajectory_msg(closing_route_, false);
    
    return track_limits_msg;

}

common_msgs::msg::Triangulation PathPlanning::create_triangulation_msg(CDT::Triangulation<double> triangulation){
    common_msgs::msg::Triangulation triangulation_msg;
    CDT::Triangulation<double>::V2dVec points = triangulation.vertices;
    CDT::TriangleVec triangles = triangulation.triangles;
    for (int i = 0; i < triangles.size(); i++){
        common_msgs::msg::Simplex simplex;
        common_msgs::msg::PointXY a, b, c;
        CDT::Triangle triangle = triangles[i];
        CDT::VerticesArr3 vertices_index = triangle.vertices;
        CDT::VertInd a_ind = vertices_index[0];
        CDT::VertInd b_ind = vertices_index[1];
        CDT::VertInd c_ind = vertices_index[2];
        a.x = points[a_ind].x;
        a.y = points[a_ind].y;
        b.x = points[b_ind].x;
        b.y = points[b_ind].y;
        c.x = points[c_ind].x;
        c.y = points[c_ind].y;
        simplex.points = {a, b, c};
        triangulation_msg.simplices.push_back(simplex);
    }
    return triangulation_msg;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}
