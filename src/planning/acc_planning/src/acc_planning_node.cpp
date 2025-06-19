/**
 * @file acc_planning_mode.cpp
 * @author David Guil (davidguilb2@gmail.com)
 * @brief Acceleration planning node for the ARUS Driverless pipeline.
 */

#include "acc_planning_node.hpp"

AccPlanning::AccPlanning() : Node("acc_planning_node")
{
    this->declare_parameter<std::string>("perception_topic", "/arussim/perception");
    this->declare_parameter<std::string>("trajectory_topic", "/acc_planning/trajectory");
    this->declare_parameter<double>("target_speed", 10.0);
    this->declare_parameter<double>("max_acc", 5.0);
    this->declare_parameter<double>("max_dec", 5.0);
    this->declare_parameter<double>("track_length", 75.0);
    this->declare_parameter<bool>("debug", true);
    this->declare_parameter<double>("step", 0.1);
    this->declare_parameter<int>("max_iterations", 500);
    this->declare_parameter<double>("ransac_threshold", 0.5);
    this->declare_parameter<int>("min_inliers_required", 3);
    this->declare_parameter<double>("max_allowed_deviation", 10.0);

    this->get_parameter("perception_topic", kPerceptionTopic);
    this->get_parameter("trajectory_topic", kTrajectoryTopic);
    this->get_parameter("target_speed", kTargetSpeed);
    this->get_parameter("max_acc", kMaxXAcc);
    this->get_parameter("max_dec", kMaxDec);
    this->get_parameter("track_length", kTrackLength);
    this->get_parameter("debug", kDebug);
    this->get_parameter("step", kStep);
    this->get_parameter("max_iterations", kMaxIterations);
    this->get_parameter("ransac_threshold", kRansacThreshold);
    this->get_parameter("min_inliers_required", kMinInliersRequired);
    this->get_parameter("max_allowed_deviation", kMaxAllowedDeviation);

    compute_profiles();

    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>(kTrajectoryTopic, 10);
    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kPerceptionTopic, 10, std::bind(&AccPlanning::perception_callback, this, std::placeholders::_1));
}


void AccPlanning::perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg) {
    auto t0 = this->now();

    cones_ = AccPlanning::convert_ros_to_pcl(per_msg);

    if (cones_.points.empty()) {
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Received empty PointCloud. Skipping iteration.");
        return;
    }

    AccPlanning::generate_planning();

    if (a_ == 0.0 && b_ == 0.0) {
        if (kDebug) RCLCPP_WARN(this->get_logger(), "No valid line found during planning. Skipping iteration.");
        return;
    } 

    AccPlanning::publish_trajectory();

    if (kDebug) RCLCPP_INFO(this->get_logger(), "Acc planning execution time: %f seconds", (this->now() - t0).seconds());
}


void AccPlanning::compute_profiles() {
    s_.clear();
    speed_profile_.clear();
    acc_profile_.clear();

    for (int i = 0; i < 2 * kTrackLength / kStep; i++) { 
        s_.push_back(i * kStep);
        speed_profile_.push_back(0.0);
    }

    speed_profile_[0] = 1;

    for (size_t i = 1; i < s_.size(); ++i) {
        double ds = kStep;
        if (s_[i] < kTrackLength) {
            speed_profile_[i] = std::min(kTargetSpeed, sqrt(pow(speed_profile_[i - 1], 2) + 2 * kMaxXAcc * ds));
        } else {
            speed_profile_[i] = std::max(0.0, sqrt(pow(speed_profile_[i - 1], 2) - 2 * kMaxDec * ds));
        }
    }

    for (size_t i = 0; i < speed_profile_.size() - 1; ++i) {
        double ds = kStep;
        acc_profile_.push_back((pow(speed_profile_[i + 1], 2) - pow(speed_profile_[i], 2)) / (2 * ds));
    }
}


void AccPlanning::generate_planning() {
    if (cones_.points.size() < kMinInliersRequired) {
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Not enough cones for line fitting.");
        return;
    }

    // Fit the first line
    
    double sum_x1 = 0.0, sum_y1 = 0.0, sum_xx1 = 0.0, sum_xy1 = 0.0;
    int n1 = cones_.points.size();

    for (const auto& cone : cones_.points) {
        sum_x1 += cone.x;
        sum_y1 += cone.y;
        sum_xx1 += cone.x * cone.x;
        sum_xy1 += cone.x * cone.y;
    }

    double mean_x1 = sum_x1 / n1;
    double mean_y1 = sum_y1 / n1;

    double a1 = (sum_xy1 - n1 * mean_x1 * mean_y1) / (sum_xx1 - n1 * mean_x1 * mean_x1);
    double b1 = mean_y1 - a1 * mean_x1;

    if (kDebug) {
        RCLCPP_INFO(this->get_logger(), "First line coefficients: a1 = %f, b1 = %f", a1, b1);
    }

    // Identify inliers for the first line
    std::vector<int> inliers_indices1;
    pcl::PointCloud<ConeXYZColorScore> remaining_cones;

    for (int idx = 0; idx < cones_.points.size(); ++idx) {
        const auto& cone = cones_.points[idx];
        double d = std::abs(a1 * cone.x + b1 - cone.y) / std::sqrt(a1 * a1 + 1);

        if (d < kRansacThreshold) {
            inliers_indices1.push_back(idx);
        } else {
            remaining_cones.push_back(cone);
        }
    }

    if (remaining_cones.size() < kMinInliersRequired) {
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Not enough remaining cones for second line fitting.");
        return;
    }

    // Fit the second line
    double sum_x2 = 0.0, sum_y2 = 0.0, sum_xx2 = 0.0, sum_xy2 = 0.0;
    int n2 = remaining_cones.size();

    for (const auto& cone : remaining_cones) {
        sum_x2 += cone.x;
        sum_y2 += cone.y;
        sum_xx2 += cone.x * cone.x;
        sum_xy2 += cone.x * cone.y;
    }

    double mean_x2 = sum_x2 / n2;
    double mean_y2 = sum_y2 / n2;

    double a2 = (sum_xy2 - n2 * mean_x2 * mean_y2) / (sum_xx2 - n2 * mean_x2 * mean_x2);
    double b2 = mean_y2 - a2 * mean_x2;

    if (kDebug) {
        RCLCPP_INFO(this->get_logger(), "Second line coefficients: a2 = %f, b2 = %f", a2, b2);
    }

    // Combine the two lines
    double a_temp = (a1 + a2) / 2;
    double b_temp = (b1 + b2) / 2;

    if (std::abs(a_temp - a1) > kMaxAllowedDeviation || std::abs(b_temp - b1) > kMaxAllowedDeviation) {
        if (kDebug) RCLCPP_WARN(this->get_logger(), "Deviation too high. Keeping initial trajectory.");
        return;
    }

    a_ = a_temp;
    b_ = b_temp;

    if (kDebug) {
        RCLCPP_INFO(this->get_logger(), "Final trajectory coefficients: a = %f, b = %f", a_, b_);
    }

    // Publish the trajectory based on the fitted line
    AccPlanning::publish_trajectory();
}


void AccPlanning::publish_trajectory() {
    common_msgs::msg::Trajectory trajectory_msg;
    
    for (size_t i = 0; i < s_.size(); ++i) {
        common_msgs::msg::PointXY point;
        point.x = i * kStep;
        point.y = a_ * point.x + b_;  // Updates the slope of the trajectory
        trajectory_msg.points.push_back(point);

        trajectory_msg.speed_profile.push_back(speed_profile_[i]);
        trajectory_msg.acc_profile.push_back(acc_profile_[i]);
    }

    trajectory_pub_->publish(trajectory_msg);
}


pcl::PointCloud<ConeXYZColorScore> AccPlanning::convert_ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud) {
    pcl::PointCloud<ConeXYZColorScore> pcl_cloud;  
    pcl::fromROSMsg(*ros_cloud, pcl_cloud); 

    return pcl_cloud;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccPlanning>());
    rclcpp::shutdown();
    return 0;
}
