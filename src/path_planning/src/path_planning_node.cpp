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
    this->get_parameter("perception_topic", kPerceptionTopic);
    this->get_parameter("triangulation_topic", kTriangulationTopic);

    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kPerceptionTopic, 10, std::bind(&PathPlanning::perception_callback, this, std::placeholders::_1));
    triangulation_pub_ = this->create_publisher<common_msgs::msg::Triangulation>(kTriangulationTopic, 10);
    midpoints_pub_ = this->create_publisher<common_msgs::msg::Simplex>("/midpoints", 10); // Temporary publisher for testing
    tree_visualization_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/tree", 10); // Temporary publisher for testing

}

/**
 * @brief Create callback function for the perception topic. Main pipeline of the path planning node.
 * 
 * @param msg Point Cloud 2 message from the perception topic.
 */
void PathPlanning::perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr per_msg)
{   
    pcl::PointCloud<ConeXYZColorScore> pcl_cloud;
    pcl::fromROSMsg(*per_msg, pcl_cloud);
    CDT::Triangulation<double> triangulation;
    triangulation = PathPlanning::create_triangulation(pcl_cloud);
    common_msgs::msg::Triangulation triangulation_msg;
    if (triangulation.isFinalized()){
        triangulation_msg = PathPlanning::create_triangulation_msg(triangulation);
        
    }
    triangulation_pub_ -> publish(triangulation_msg);
    vertices_ = triangulation.vertices;
    int vertices_size = vertices_.size();
    triangles_ = triangulation.triangles;
    max_index_ = triangles_.size();
    std::vector<std::vector<std::vector<int>>> routes;
    //% TEST AREA
    //// simplex_tree triangulation_tree = PathPlanning::create_triangulation_tree(triangulation);
    CDT::NeighborsArr3 neig = triangles_[0].neighbors;
    CDT::TriInd n = neig[0];
    CDT::VerticesArr3 v = triangles_[n].vertices;
    CDT::V2d<double> vert = vertices_[v[0]];
    //% END TEST AREA

    std::vector<CDT::V2d<double>> midpoints = PathPlanning::get_midpoints(triangulation);
    common_msgs::msg::Simplex midpoints_msg;

    //% TEST AREA
    for (std::vector<CDT::V2d<double> >::size_type i = 0; i <midpoints.size(); i++){
        common_msgs::msg::PointXY point;
        point.x = midpoints[i].x;
        point.y = midpoints[i].y;
        midpoints_msg.points.push_back(point);
    }
    midpoints_pub_ -> publish(midpoints_msg); 
    //% END TEST AREA

    closest_midpoint_ = PathPlanning::get_closest_midpoint(midpoints);
    closest_triangle_ind_ = PathPlanning::get_closest_triangle();

    std::cout << "max_index " << max_index_ << std::endl;

    int orig_index = PathPlanning::get_orig_index();
    std::vector<int> o_triangles = PathPlanning::get_triangles_from_vert(orig_index);
    std::vector<SimplexTree> trees;
    
    for (int i = 0; i<o_triangles.size(); i++){
        SimplexTree tree(triangles_, vertices_, o_triangles[i], o_triangles);
        trees.push_back(tree);
        routes.push_back(tree.routes);
    }

    for (const auto &tree_routes : routes){
        for (const auto &route : tree_routes){
            for (const auto &element : route){
                std::cout << element << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "------" << std::endl;
    }

    // PathPlanning::visualize_tree(routes[0]);
    // PathPlanning::visualize_tree(routes[1]);
}

CDT::Triangulation<double> PathPlanning::create_triangulation(pcl::PointCloud<ConeXYZColorScore> input_cloud){
    CDT::Triangulation<double> triangulation;
    std::vector<CDT::V2d<double>> points;
    for (std::size_t i = 0; i<input_cloud.size();i++){
        ConeXYZColorScore cone = input_cloud.points[i];
        CDT::V2d<double> point = CDT::V2d<double>::make(cone.x, cone.y);
        points.push_back(point);
    }
    points.push_back(CDT::V2d<double>::make(0,0));
    triangulation.insertVertices(points);
    triangulation.eraseSuperTriangle();
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
    double min_norm = PathPlanning::norm(closest);

    for (std::vector<CDT::V2d<double>>::iterator it = midpoint_arr.begin(); it != midpoint_arr.end(); it++){
        double current_norm = PathPlanning::norm(*it);
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
        CDT::V2d<double> centroid = PathPlanning::compute_centroid(i);
        double current_norm = PathPlanning::norm(centroid);
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

int PathPlanning::get_orig_index(){
    CDT::V2d<double> o = CDT::V2d<double>::make(0,0);
    int o_ind;
    for (int i = 0; i<vertices_.size(); i++){
        if (vertices_[i] == o){
            o_ind = i;
            return o_ind;
        }
    }
    //% Borrar cuando se depure
    std::cout << "Algo mal (index orig)" << std::endl;
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

//% TEST AREA
void PathPlanning::visualize_tree(std::vector<std::vector<int>> routes){
    visualization_msgs::msg::MarkerArray marker_array;
    int i = 0;
    for (const auto &route : routes){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "arussim/vehicle_cog";
        marker.ns = "routes";
        marker.id = i;
        i++;
        marker.type = 4;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 0.7;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        for (const auto &id : route){
            CDT::V2d<double> p = PathPlanning::compute_centroid(id);
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0;
            marker.points.push_back(point);
        }
        marker_array.markers.push_back(marker);
    }
    tree_visualization_pub->publish(marker_array);
    
}
//% END TEST AREA


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}