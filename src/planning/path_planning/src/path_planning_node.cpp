/**
 * @file path_planning_node.cpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @brief Main file for the Path Planning node. Contains the main function and the implementation
 * of the methods to achieve a robust and reliable path planning algorithm for the ARUS Team
 * which extracts the midpoints of the track that the ART will follow.
 * @version 0.1
 * @date 29-10-2024
 *
 */
#include "path_planning/path_planning_node.hpp"

PathPlanning::PathPlanning() : Node("path_planning")
{
    // Declare and get parameters
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
    this->declare_parameter<double>("max_tri_angle", 2.9);
    
    this->get_parameter("max_tri_len", kMaxTriLen);
    this->get_parameter("max_tri_angle", kMaxTriAngle);


    // Route
    this->declare_parameter<double>("angle_coeff", 1.0);
    this->declare_parameter<double>("len_coeff", 1.0);
    this->declare_parameter<double>("smooth_coeff", 1.2);
    this->declare_parameter<double>("prev_route_bias", 0.75);
    this->declare_parameter<int>("route_back", 10);
    this->declare_parameter<bool>("use_buffer", false);
    this->declare_parameter<bool>("use_closing_route", false);
    this->declare_parameter<bool>("stop_after_closing", false);

    this->get_parameter("angle_coeff", kAngleCoeff);
    this->get_parameter("len_coeff", kLenCoeff);
    this->get_parameter("smooth_coeff", kSmoothCoeff);
    this->get_parameter("prev_route_bias", kPrevRouteBias);
    this->get_parameter("route_back", kRouteBack);
    this->get_parameter("use_buffer", kUseBuffer);
    this->get_parameter("use_closing_route", kUseClosingRoute);
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
    if (kStopAfterClosing && (closing_route_.size() > 0)){
        // Publish the closing route
        trajectory_pub_ -> publish(this->create_trajectory_msg(closing_route_));
        if (lap_count_ == 0) return;
    }

    // Save the point cloud as a pcl object from ROS2 msg
    pcl::fromROSMsg(*per_msg, pcl_cloud_);

    // Check if the point cloud is empty and return if it is
    if(pcl_cloud_.size() < 3){
        RCLCPP_INFO(this->get_logger(), "Empty point cloud");
        return;
    }

    if (lap_count_ == 0){
        // In the first lap add current car position as the origin of the tree
        pcl_cloud_.push_back(origin_);
    } else {
        // In the second lap add the origin (0,0) as the origin of the tree
        pcl_cloud_.push_back(ConeXYZColorScore(0, 0, 0, UNCOLORED, 1));
    }
    
    // Create the triangulation
    CDT::Triangulation<double> triangulation;
    triangulation = this->create_triangulation(pcl_cloud_);

    // Publish the triangulation as a ROS2 message
    common_msgs::msg::Triangulation triangulation_msg;
    if (triangulation.isFinalized()){
        triangulation_msg = this->create_triangulation_msg(triangulation);
    }
    triangulation_pub_ -> publish(triangulation_msg);

    // Get attributes from the triangulation
    vertices_ = triangulation.vertices;
    triangles_ = triangulation.triangles;

    // Construct the tree from the triangulation. Initializing it from each of the origin triangles
    int orig_index;
    if (lap_count_ == 0){
        orig_index = this->get_vertex_index(CDT::V2d<double>::make(x_,y_));
    } else {
        orig_index = this->get_vertex_index(CDT::V2d<double>::make(0,0));
    }
    std::vector<int> o_triangles = this->get_triangles_from_vert(orig_index);

    // Get the best route from the trees
    SimplexTree tree;
    double min_cost = INFINITY;
    for (int i = 0; i<o_triangles.size(); i++){
        // Calculate the centroid of the triangle and the angle difference with the car yaw
        CDT::V2d<double> centroid = compute_centroid(o_triangles[i], triangles_, vertices_);
        double angle_diff;
        if (lap_count_ == 0){
            angle_diff = abs(atan2(centroid.y-y_, centroid.x-x_)-yaw_);
        } else {
            angle_diff = abs(atan2(centroid.y, centroid.x));
        }
        double corrected_angle_diff = std::min(angle_diff, 2*M_PI-angle_diff);

        // Create the tree if the angle difference is less than pi/3 degrees
        if (corrected_angle_diff <  M_PI/2){
            tree = SimplexTree(triangles_, o_triangles[i], orig_index, pcl_cloud_, yaw_,
                             kAngleCoeff, kLenCoeff);
            if (tree.min_cost_ < min_cost){
                min_cost = tree.min_cost_;
                best_midpoint_route_ = tree.best_route_;
                best_index_route_ = tree.best_index_route_;
            }
        }
    }

    // Get the track limits in the second lap
    if (lap_count_ > 0 && x_>3 && !track_limits_sent_){
        track_limits_pub_ -> publish(this->create_track_limits_msg(best_index_route_));
        track_limits_sent_ = true;
    }

    // Get the final route using the buffer if selected in config file
    std::vector<ConeXYZColorScore> final_route;

    if (kUseBuffer){
        previous_midpoint_routes_.push_back(best_midpoint_route_);
        final_route = this->get_final_route();
    } else {
        final_route = best_midpoint_route_;
    }

    if (final_route.size() <3) return;

    if ((kUseClosingRoute and tree.end_) || lap_count_ > 0){
        closing_route_ = final_route;
    }

    // Publish the best trajectory
    trajectory_pub_ -> publish(this->create_trajectory_msg(final_route));

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
    origin_ = ConeXYZColorScore(x_, y_, 0, UNCOLORED, 1);
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
        a_angle = acos((ab.x*ca.x+ab.y*ca.y)/(norm(ab)*norm(ca)));
        b_angle = acos((bc.x*ab.x+bc.y*ab.y)/(norm(bc)*norm(ab)));
        c_angle = acos((ca.x*bc.x+ca.y*bc.y)/(norm(ca)*norm(bc)));

        // Delete triangles with long edges or big angles (except the ones with the origin vertex)
        // and triangles with the same color
        if (lap_count_==0 && ((a.x == origin_.x and a.y == origin_.y) or     // Check if the origin vertex is
                                (b.x == origin_.x and b.y == origin_.y) or    // in the triangle and skip it
                                (c.x == origin_.x and c.y == origin_.y))){     // (origin vertex is the car position)
            continue;
        }

        if (a.color != UNCOLORED and a.color == b.color and b.color == c.color){
            deleted_tri.insert(i);
        } else if (distance(a,b) > kMaxTriLen or distance(b,c) > kMaxTriLen or distance(c,a) > kMaxTriLen){
            deleted_tri.insert(i);
        } else if (a_angle > kMaxTriAngle or b_angle > kMaxTriAngle or c_angle > kMaxTriAngle){
            deleted_tri.insert(i);
        }
    }
    triangulation.removeTriangles(deleted_tri);
    return triangulation;
}

common_msgs::msg::Triangulation PathPlanning::create_triangulation_msg(CDT::Triangulation<double> triangulation){
    common_msgs::msg::Triangulation triangulation_msg;
    CDT::Triangulation<double>::V2dVec points = triangulation.vertices;
    CDT::TriangleVec triangles = triangulation.triangles;
    for (std::vector<CDT::Triangle>::size_type i = 0; i < triangles.size(); i++){
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

double PathPlanning::norm(CDT::V2d<double> v){
    return hypot(v.x, v.y);
}

int PathPlanning::get_vertex_index(CDT::V2d<double> vertex){
    int o_ind;
    for (int i = 0; i<vertices_.size(); i++){
        if (vertices_[i] == vertex){
            o_ind = i;
            return o_ind;
        }
    }
    return 0;
}

std::vector<int> PathPlanning::get_triangles_from_vert(int vert_index){
    std::vector<int> o_triangles;
    for (int i = 0; i<triangles_.size(); i++){
        if (triangles_[i].containsVertex(vert_index)){
            o_triangles.push_back(i);
        }
    }
    return o_triangles;
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



common_msgs::msg::Trajectory PathPlanning::create_trajectory_msg(std::vector<ConeXYZColorScore> route){
    int route_size = route.size();
    int degree = 2;
    common_msgs::msg::Trajectory trajectory_msg;
    double acum = 0.0;
    trajectory_msg.s = {0.0};

    std::vector<double> xp, yp, xpp, ypp, v_grip, s, k, speed_profile, acc_profile;

    if (route_size == 0){
        return trajectory_msg;
    } else if (route_size < 3){
        common_msgs::msg::PointXY point;
        point.x = route[0].x;
        point.y = route[0].y;
        trajectory_msg.points.push_back(point);
        return trajectory_msg;
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
        if (den == 0){
            val = 0;
        } else{
            val = (xp[i]*ypp[i]-yp[i]*xpp[i])/den;
        }
        k.push_back(val);
    }

    for (const auto &val : k){
        double v = std::min(kMaxVel, sqrt(kMaxYAcc/std::max(abs(val), 0.0001)));
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

    for (int i = 0; i<speed_profile.size()-1; i++){
        double ds = s[i+1]-s[i];
        if (ds != 0){
            acc_profile.push_back((pow(speed_profile[i+1], 2)-pow(speed_profile[i], 2))/(2*ds));
        }
    }

    for (int i = 0; i<s.size(); i++){
        trajectory_msg.k.push_back(k[i]);
        trajectory_msg.s.push_back(s[i]);
        trajectory_msg.speed_profile.push_back(speed_profile[i]);
        trajectory_msg.acc_profile.push_back(acc_profile[i]);
    }

    return trajectory_msg;
}

common_msgs::msg::TrackLimits PathPlanning::create_track_limits_msg(std::vector<int> triangles_route){
    CDT::Triangle last_triangle = triangles_[triangles_route.back()];
    CDT::NeighborsArr3 neighbors = last_triangle.neighbors;
    for (int i = 0; i<3; i++){
        if (neighbors[i] < triangles_.size() && !in(neighbors[i], triangles_route)){
            triangles_route.push_back(neighbors[i]);
        }
    }
    std::vector<int> left_limit, right_limit;
    for (int i = 0; i<triangles_route.size(); i++){
        CDT::Triangle triangle = triangles_[triangles_route[i]];
        CDT::VerticesArr3 vertices = triangle.vertices;
        for (int j = 0; j<3; j++){
            ConeXYZColorScore cone = pcl_cloud_.points[vertices[j]];
            int vertex = vertices[j];
            switch (cone.color){
            case BLUE:
                if (!in(vertex, left_limit)){
                    left_limit.push_back(vertex);
                }
                break;
            case YELLOW:
                if (!in(vertex, right_limit)){
                    right_limit.push_back(vertex);
                }
                break;
            default:
                break;
            }
        }
    }
    common_msgs::msg::TrackLimits track_limits_msg;
    for (int i = 0; i<left_limit.size(); i++){
        common_msgs::msg::PointXY point;
        point.x = pcl_cloud_.points[left_limit[i]].x;
        point.y = pcl_cloud_.points[left_limit[i]].y;
        track_limits_msg.left_limit.push_back(point);
    }
    for (int i = 0; i<right_limit.size(); i++){
        common_msgs::msg::PointXY point;
        point.x = pcl_cloud_.points[right_limit[i]].x;
        point.y = pcl_cloud_.points[right_limit[i]].y;
        track_limits_msg.right_limit.push_back(point);
    }
    track_limits_msg.trajectory = this->create_trajectory_msg(best_midpoint_route_);
    
    return track_limits_msg;

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}