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
#include <cmath>

PathPlanning::PathPlanning() : Node("path_planning")
{
    this->declare_parameter<std::string>("perception_topic", "/perception");
    this->declare_parameter<std::string>("triangulation_topic", "/triangulation"); 
    this->declare_parameter<std::string>("trajectory_topic", "/trajectory");
    this->declare_parameter<double>("max_tri_len", 7);
    this->declare_parameter<double>("max_tri_angle", 2.9);
    this->declare_parameter<double>("len_coeff", 1.0);
    this->declare_parameter<double>("angle_coeff", 1.0);
    this->declare_parameter<double>("max_angle", M_PI);
    this->declare_parameter<double>("v_max", 10.0);
    this->declare_parameter<double>("ay_max", 5.0);
    this->declare_parameter<double>("ax_max", 5.0);
    this->declare_parameter<double>("smooth", 0.3);
    this->get_parameter("perception_topic", kPerceptionTopic);
    this->get_parameter("triangulation_topic", kTriangulationTopic);
    this->get_parameter("trajectory_topic", kTrajectoryTopic);
    this->get_parameter("max_tri_len", kMaxTriLen);
    this->get_parameter("max_tri_angle", kMaxTriAngle);
    this->get_parameter("len_coeff", kLenCoeff);
    this->get_parameter("angle_coeff", kAngleCoeff);
    this->get_parameter("max_angle", kMaxAngle);
    this->get_parameter("v_max", kMaxVel);
    this->get_parameter("ay_max", kMaxYAcc);
    this->get_parameter("ax_max", kMaxXAcc);
    this->get_parameter("smooth", kSmooth);

    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kPerceptionTopic, 10, std::bind(&PathPlanning::perception_callback, this, std::placeholders::_1));
    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
        "/car_state/state", 10, std::bind(&PathPlanning::car_state_callback, this, std::placeholders::_1));
    triangulation_pub_ = this->create_publisher<common_msgs::msg::Triangulation>(kTriangulationTopic, 10);
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>(kTrajectoryTopic, 10);

}

void PathPlanning::perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr per_msg)
{   
    // Conform triangulation from the point cloud
    pcl::PointCloud<ConeXYZColorScore> pcl_cloud;
    pcl::fromROSMsg(*per_msg, pcl_cloud);

    if(pcl_cloud.size() == 0){
        RCLCPP_INFO(this->get_logger(), "Empty point cloud");
        return;
    }

    CDT::Triangulation<double> triangulation;
    triangulation = this->create_triangulation(pcl_cloud);
    common_msgs::msg::Triangulation triangulation_msg;
    if (triangulation.isFinalized()){
        triangulation_msg = this->create_triangulation_msg(triangulation);
    }
    triangulation_pub_ -> publish(triangulation_msg);

    // Get attributes from the triangulation
    vertices_ = triangulation.vertices;
    triangles_ = triangulation.triangles;

    // Construct the tree from the triangulation
    int orig_index = this->get_vertex_index(CDT::V2d<double>::make(x_,y_));
    std::vector<int> o_triangles = this->get_triangles_from_vert(orig_index);
    std::vector<SimplexTree> trees;
    triangle_routes_ = {};
    for (int i = 0; i<o_triangles.size(); i++){
        SimplexTree tree(triangles_, o_triangles[i], o_triangles);
        trees.push_back(tree);
        for (int j = 0; j<tree.index_routes.size(); j++){
            triangle_routes_.push_back(tree.index_routes[j]);
        }
    }

    // Get the midpoints routes from triangles routes
    midpoint_routes_ = {};
    this->get_midpoint_routes();

    // Get the cost of each route
    if(midpoint_routes_.size()>0){
        int best_route_ind = 0;
        double min_cost = INFINITY;
        for (int i = 0; i<midpoint_routes_.size(); i++){
            double cost = this->get_route_cost(midpoint_routes_[i]);
            if (cost < min_cost){
                min_cost = cost;
                best_route_ind = i;
            }
        }
        best_midpoint_route_ = midpoint_routes_[best_route_ind];

        // Publish the best trajectory
        trajectory_pub_ -> publish(this->create_trajectory_msg(best_midpoint_route_));

    }
}

void PathPlanning::car_state_callback(const common_msgs::msg::State::SharedPtr state_msg)
{
    x_ = state_msg->x;
    y_ = state_msg->y;
    yaw_ = state_msg->yaw;
    vx_ = state_msg->vx;
    vy_ = state_msg->vy;
    v_ = hypot(vx_, vy_);
}

CDT::Triangulation<double> PathPlanning::create_triangulation(pcl::PointCloud<ConeXYZColorScore> input_cloud){
    CDT::Triangulation<double> triangulation;
    std::vector<CDT::V2d<double>> points;
    for (std::size_t i = 0; i<input_cloud.size();i++){
        ConeXYZColorScore cone = input_cloud.points[i];
        CDT::V2d<double> point = CDT::V2d<double>::make(cone.x, cone.y);
        points.push_back(point);
    }
    CDT::V2d<double> origin = CDT::V2d<double>::make(x_,y_);
    points.push_back(origin);
    triangulation.insertVertices(points);
    triangulation.eraseSuperTriangle();
    CDT::TriangleVec triangles = triangulation.triangles;
    CDT::Triangulation<double>::V2dVec vertices = triangulation.vertices;
    CDT::TriIndUSet deleled_tri;
    for (int i=0; i<triangles.size(); i++) {
        // Angle and length calculations
        CDT::VerticesArr3 vert_ind = triangles[i].vertices;
        CDT::V2d<double> a = vertices[vert_ind[0]];
        CDT::V2d<double> b = vertices[vert_ind[1]];
        CDT::V2d<double> c = vertices[vert_ind[2]];
        CDT::V2d<double> ab, bc, ca;
        double a_angle, b_angle, c_angle;
        ab = CDT::V2d<double>::make(b.x-a.x, b.y-a.y);
        bc = CDT::V2d<double>::make(c.x-b.x, c.y-b.y);
        ca = CDT::V2d<double>::make(a.x-c.x, a.y-c.y);
        a_angle = acos((ab.x*ca.x+ab.y*ca.y)/(norm(ab)*norm(ca)));
        b_angle = acos((bc.x*ab.x+bc.y*ab.y)/(norm(bc)*norm(ab)));
        c_angle = acos((ca.x*bc.x+ca.y*bc.y)/(norm(ca)*norm(bc)));

        // Delete triangles with long edges or big angles (except the ones with the origin vertex)
        if (CDT::distance(a,b) > kMaxTriLen or CDT::distance(b,c) > kMaxTriLen or CDT::distance(c,a) > kMaxTriLen){
            deleled_tri.insert(i);
        } else if (a == origin or b == origin or c == origin){
            continue;
        } else if (a_angle > kMaxTriAngle or b_angle > kMaxTriAngle or c_angle > kMaxTriAngle){
            deleled_tri.insert(i);
        }
    }
    triangulation.removeTriangles(deleled_tri);
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

std::vector<CDT::V2d<double>> PathPlanning::get_midpoints(CDT::Triangulation<double> triangulation){
    std::vector<CDT::V2d<double>> midpoints;
    CDT::TriangleVec triangles = triangulation.triangles;
    CDT::Triangulation<double>::V2dVec vertices = triangulation.vertices;

    for (std::vector<CDT::Triangle>::size_type i = 0; i < triangles.size(); i++){
        CDT::Triangle triangle = triangles[i];
        CDT::VerticesArr3 vertices_index = triangle.vertices;
        CDT::VertInd a_ind = vertices_index[0];
        CDT::VertInd b_ind = vertices_index[1];
        CDT::VertInd c_ind = vertices_index[2];
        CDT::V2d<double> a = vertices[a_ind];
        CDT::V2d<double> b = vertices[b_ind];
        CDT::V2d<double> c = vertices[c_ind];
        CDT::V2d<double> ab = CDT::V2d<double>::make((a.x+b.x)/2, (a.y+b.y)/2);
        CDT::V2d<double> bc = CDT::V2d<double>::make((b.x+c.x)/2, (b.y+c.y)/2);
        CDT::V2d<double> ca = CDT::V2d<double>::make((c.x+a.x)/2, (c.y+a.y)/2);
        midpoints.push_back(ab);
        midpoints.push_back(bc);
        midpoints.push_back(ca);
    }
    CDT::RemoveDuplicates(midpoints);

    return midpoints;
}

double PathPlanning::norm(CDT::V2d<double> v){
    return hypot(v.x, v.y);
}

CDT::V2d<double> PathPlanning::get_closest_midpoint(std::vector<CDT::V2d<double>> midpoint_arr){
    CDT::V2d<double> closest = midpoint_arr[0];
    double min_norm = this->norm(closest);

    for (std::vector<CDT::V2d<double>>::iterator it = midpoint_arr.begin(); it != midpoint_arr.end(); it++){
        double current_norm = this->norm(*it);
        if (current_norm < min_norm){
            min_norm = current_norm;
            closest = *it;
        }
    }
    return closest;
}

int PathPlanning::get_closest_triangle(){
    int closest = 0;
    double min_norm = 100;
    for (int i = 0; i < triangles_.size(); i++){
        CDT::V2d<double> centroid = this->compute_centroid(i);
        double current_norm = this->norm(centroid);
        if (current_norm < min_norm){
            min_norm = current_norm;
            closest = i;
        }
    }
    return closest;
}

CDT::V2d<double> PathPlanning::compute_centroid(int triangle_ind){
    CDT::V2d<double> a, b, c;
    CDT::Triangle triangle = triangles_[triangle_ind];
    CDT::VerticesArr3 vertices_index = triangle.vertices;
    CDT::VertInd a_ind = vertices_index[0];
    CDT::VertInd b_ind = vertices_index[1];
    CDT::VertInd c_ind = vertices_index[2];
    a = vertices_[a_ind];
    b = vertices_[b_ind];
    c = vertices_[c_ind];
    CDT::V2d<double> centroid = CDT::V2d<double>::make((a.x+b.x+c.x)/3, (a.y+b.y+c.y)/3);
    return centroid;
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

CDT::Edge PathPlanning::get_share_edge(CDT::Triangle triangle1, CDT::Triangle triangle2){
    CDT::VerticesArr3 vertices1 = triangle1.vertices;
    CDT::VerticesArr3 vertices2 = triangle2.vertices;
    std::vector<int> shared_vertices = {};
    for (int i = 0; i<3; i++){
        if (std::find(vertices2.begin(), vertices2.end(), vertices1[i])!=vertices2.end()){
            shared_vertices.push_back(vertices1[i]);
        };
    }
    CDT::Edge shared_edge(shared_vertices[0], shared_vertices[1]);
    return shared_edge;
}

void PathPlanning::get_midpoint_routes(){
    for (const auto &ind_route : triangle_routes_){
        std::vector<CDT::V2d<double>> mid_route = {CDT::V2d<double>::make(x_,y_)}; // Start from the car position
        for (int i = 0; i < ind_route.size()-1; i++){
            CDT::Triangle triangle = triangles_[ind_route[i]];
            CDT::Triangle next_triangle = triangles_[ind_route[i+1]];
            CDT::Edge share_edge = this->get_share_edge(triangle, next_triangle);
            CDT::VertInd v1 = share_edge.v1();
            CDT::VertInd v2 = share_edge.v2();
            CDT::V2d<double> midpoint = CDT::V2d<double>::make((vertices_[v1].x+vertices_[v2].x)/2, 
                                                               (vertices_[v1].y+vertices_[v2].y)/2);
            mid_route.push_back(midpoint);
        }
        midpoint_routes_.push_back(mid_route);
    }
}

double PathPlanning::get_route_cost(std::vector<CDT::V2d<double>> &route){
    // Initialize the cost to 0
    double route_cost = 0;
    int route_size = route.size();
    if (route_size < 3){
        return INFINITY;
    }
    // Check if the route is looking forward (angle difference with the yaw is less than pi/2 radians)
    bool route_looking_forward = abs(atan2(route[1].y-y_, route[1].x-x_)-yaw_) < M_PI/2;
    if (!route_looking_forward){
        return INFINITY;
    }

    // Initialize the properties of the route
    double route_length = CDT::distance(route[0], route[1]);
    double angle_diff_sum = 0;

    // Store the route while iterating
    std::vector<CDT::V2d<double>> route_out = {CDT::V2d<double>::make(x_, y_)};
    
    // Iterate over the route and calculate the cost
    for (int i = 0; i<route_size-2;i++){
        route_length += CDT::distance(route[i+1], route[i+2]);
        double angle_diff = abs(atan2(route[i+2].y-route[i+1].y, route[i+2].x-route[i+1].x)-
                                atan2(route[i+1].y-route[i].y, route[i+1].x-route[i].x));
        double corrected_angle_diff = std::min(angle_diff, 2*M_PI-angle_diff);
        route_out.push_back(route[i+1]);

        // If the angle is too big, cut the route and return the cost
        if (corrected_angle_diff > kMaxAngle){
            angle_diff_sum += angle_diff;
            route = route_out;           // Cut the route
            return kAngleCoeff*angle_diff_sum-kLenCoeff*route_length;
        } else{
            angle_diff_sum += angle_diff;
        }
    }

    route_cost += kAngleCoeff*angle_diff_sum - kLenCoeff*route_length; // Curvature
    return route_cost;
}

common_msgs::msg::Trajectory PathPlanning::create_trajectory_msg(std::vector<CDT::V2d<double>> route){
    int route_size = route.size();
    int degree = 2 ;
    common_msgs::msg::Trajectory trajectory_msg;
    double acum = 0.0;
    trajectory_msg.s = {0.0};

    std::vector<double> xp, yp, xpp, ypp, v_grip, s, k, speed_profile, acc_profile;
    std::vector<double> t_coords, x_coords, y_coords;

    if (route_size < 3){
        common_msgs::msg::PointXY point;
        point.x = route[0].x;
        point.y = route[0].y;
        trajectory_msg.points.push_back(point);
        return trajectory_msg;
    }

    Eigen::MatrixXd control_points(2, route_size);
    for (int i = 0; i<route_size; i++){
        control_points(0, i) = route[i].x;
        control_points(1, i) = route[i].y;
    }

    Eigen::Spline<double, 2> bSpline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(
        control_points, degree);
    
    // Add the points to the trajectory message
    for (double i = 0; i<route_size*10-1; i++){
        Eigen::Vector2d point_1 = bSpline(i/(10*route_size));
        Eigen::Vector2d point_2 = bSpline((i+1)/(10*route_size));
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

    speed_profile[0] = v_;

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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}