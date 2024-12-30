#include <rclcpp/rclcpp.hpp>

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include "common_msgs/msg/state.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "ConeXYZColorScore.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "graph_slam/data_association_euclidean.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>



class GraphSlam : public rclcpp::Node
{
  public:
    GraphSlam();

  private:

    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Matrix3d B = Eigen::Matrix3d::Identity();
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    double dt = 0.01;

    DataAssociation DA;

    rclcpp::CallbackGroup::SharedPtr collector_callback_group_;
    rclcpp::CallbackGroup::SharedPtr optimizer_callback_group_;
    
    Eigen::Vector2d local_to_global(const Eigen::Vector2d& position);

    Eigen::Vector3d vehicle_pose_ = Eigen::Vector3d::Zero();
    // TODO add configuration for noise
    double x_noise_ = 1.0E-5;
    double y_noise_ = 1.0E-5;
    double yaw_noise_ = 5.0E-7;
    double perception_noise_ = 0.01;

    g2o::SparseOptimizer optimizer_;
    std::vector<g2o::VertexSE2*> pose_vertices_; // All pose vertices
    std::vector<g2o::VertexPointXY*> landmark_vertices_; // All landmark vertices
    std::vector<g2o::EdgeSE2*> pose_edges_; // All pose edges
    std::vector<g2o::EdgeSE2PointXY*> landmark_edges_; // All landmark edges
    std::vector<g2o::HyperGraph::Vertex*> vertices_to_add_; // Vertices to add to the optimizer
    std::vector<g2o::HyperGraph::Edge*> edges_to_add_; // Edges to add to the optimizer

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		rclcpp::Time prev_t_;

    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;
    rclcpp::TimerBase::SharedPtr optimizer_timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

    void state_callback(const common_msgs::msg::State::SharedPtr msg);
    void perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void optimizer_callback();
    void addVerticesAndEdges();
    void send_tf();
    void publish_map();
    void update_data_association_map();

};