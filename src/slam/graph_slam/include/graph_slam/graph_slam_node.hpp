/**
 * @file graph_slam_node.hpp
 * @author Álvaro Landero (alplepe02@gmail.com)
 * @brief GraphSlam node header for ARUS Team Driverless pipeline
 */

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
#include <g2o/core/robust_kernel_impl.h>

#include "common_msgs/msg/state.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int16.hpp>

#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>
#include <cstdlib> 
#include <filesystem>
#include "ConeXYZColorScore.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "graph_slam/data_association_ICP_NN.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>



class GraphSlam : public rclcpp::Node
{
  public:
    GraphSlam();

  private:

    // Odometry variables
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Matrix3d B = Eigen::Matrix3d::Identity();
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    Eigen::Vector3d vehicle_pose_ = Eigen::Vector3d::Zero();
    double x_noise_ = 1.0E-5;
    double y_noise_ = 1.0E-5;
    double yaw_noise_ = 5.0E-7;
    double perception_noise_ = 1.0E-5;
    double dt = 0.01;
    g2o::VertexSE2* last_optimized_pose_ = nullptr;
    std::vector<Eigen::Vector3d> odom_buffer_;

    // Graph variables
    DataAssociation DA;
    g2o::SparseOptimizer optimizer_;
    std::vector<g2o::VertexSE2*> pose_vertices_; // All pose vertices
    std::vector<g2o::VertexPointXY*> landmark_vertices_; // All landmark vertices
    std::vector<g2o::EdgeSE2*> pose_edges_; // All pose edges
    std::vector<g2o::EdgeSE2PointXY*> landmark_edges_; // All landmark edges
    std::vector<g2o::VertexSE2*> pose_vertices_to_add_; // Vertices to add to the optimizer
    std::vector<g2o::VertexPointXY*> landmark_vertices_to_add_; // Vertices to add to the optimizer
    std::vector<g2o::HyperGraph::Edge*> edges_to_add_; // Edges to add to the optimizer

    // Other variables
    double driven_distance_ = 0.0;
    int lap_count_ = 0;
    bool map_fixed_ = false;
    int pose_edges_deactivated_ = 0;
    int landmark_edges_deactivated_ = 0;
		rclcpp::Time prev_t_;

    // Parameters
    std::string kMapTopic;
    std::string kFinalMapTopic;
    std::string kCarStateTopic;
    std::string kLapCountTopic;
    std::string kGlobalFrame;
    std::string kLocalFrame;
    std::string kMapFrame;
    double kOptimizerFreq;
    double kFinishLineOffset;
    double kTrackWidth;
    double kMinLapDistance;
    int kMaxPoseEdges;
    int kMaxLandmarkEdges;
    bool kDebug;
    double kPosLidarX;
    std::string kPerceptionTopic;

    rclcpp::CallbackGroup::SharedPtr collector_callback_group_;
    rclcpp::CallbackGroup::SharedPtr optimizer_callback_group_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timers
    rclcpp::TimerBase::SharedPtr optimizer_timer_;

    // Subscribers
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr final_map_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr lap_count_pub_;

    /**
     * @brief Callback function for the optimizer timer
     */
    void optimizer_callback();

    /**
     * @brief Callback function for the state subscriber
     */
    void state_callback(const common_msgs::msg::State::SharedPtr msg);

    /**
     * @brief Callback function for the perception subscriber
     */
    void perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief Load vertices and edges to add to the optimizer
     */
    void fill_graph();

    /**
     * @brief Fix all landmarks in the graph
     */
    void fix_map();

    /**
     * @brief Send frame transform to tf2
     */
    void send_tf();

    /**
     * @brief Publish the map as a PointCloud2 message
     */
    void publish_map();

    /**
     * @brief Check if the vehicle has crossed the finish line
     */
    void check_finish_line();

    /**
     * @brief Update the world position of the landmarks in the data association map after optimization
     * Landmarks must be pointers to be updated
     */
    void update_data_association_map();

    /**
     * @brief Update vehicle_pose after optimization
     */
    void update_pose_predictions();

    /**
     * @brief Get the global position given the local position relative to the actual vehicle pose
     */
    Eigen::Vector2d local_to_global(const Eigen::Vector2d& local_pos);

    /**
     * @brief Get the local position relative to the actual vehicle pose given the global position
     */
    Eigen::Vector2d global_to_local(const Eigen::Vector2d& global_pos);

};