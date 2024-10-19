/**
 * @class pure_pursuit
 * @brief pure_pursuit class 
 * 
 * Controller pp
 */
#include <vector>
#include <limits>
#include <cmath>
#include "common_msgs/msg/point_xy.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h> 
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <algorithm>

/**
 * @class pure_pursuit
 * @brief pure_pursuit class 
 * 
 * Controller pp
 */

class PurePursuit {
public:
    // Constructor
    PurePursuit(rclcpp::Node::SharedPtr node);

    // Update the path with a new trajectory
    void update_path(const std::vector<common_msgs::msg::PointXY>& new_path);

    // Search for the pursuit point based on the car's current position
    common_msgs::msg::PointXY search_pursuit_point(
        double look_ahead_distance, const common_msgs::msg::PointXY& car_position);

    // Calculate the steering angle to follow the path
    double get_steering_angle();

private:
    // Variables to store the trajectory and internal states
    std::vector<common_msgs::msg::PointXY> path;
    std::vector<double> distance_along_curve;
    bool path_updated = false;
    size_t pursuit_index = 0;

    // Configuration parameters
    std::string global_frame;
    std::string car_frame;
    bool global_mode = true;
    double look_ahead_distance = 1.0;  // Default look-ahead distance

    // Car state and ROS integration
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    double prev_steer = 0.0;

    // Temporary variables for car position and orientation
    common_msgs::msg::PointXY car_position;
};

// Constructor implementation
PurePursuit::PurePursuit(rclcpp::Node::SharedPtr node) 
    : node_(node), 
    tf_buffer_(node->get_clock()),  
    tf_listener_(tf_buffer_) {
        
    // Load parameters from the ROS server
    node_->declare_parameter("global_frame", "map");
    node_->declare_parameter("car_frame", "base_link");
    node_->declare_parameter("global_mode", true);
    node_->declare_parameter("LAD", look_ahead_distance); // LAD is look-ahead distance

    node_->get_parameter("global_frame", global_frame);
    node_->get_parameter("car_frame", car_frame);
    node_->get_parameter("global_mode", global_mode);
    node_->get_parameter("LAD", look_ahead_distance);
}

// Implementation of update_path
void PurePursuit::update_path(const std::vector<common_msgs::msg::PointXY>& new_path) {
    // Check if the new path is empty
    if (new_path.empty()) {
        return; // If path is empty, keep the last valid path
    }

    path = new_path;
    path_updated = true;
}

// Implementation of search_pursuit_point
common_msgs::msg::PointXY PurePursuit::search_pursuit_point(double look_ahead_distance, const common_msgs::msg::PointXY& car_position) {
    size_t closest_point_index = 0;
    double min_distance = std::numeric_limits<double>::infinity();

    // Clear distances if the path has been updated
    if (path_updated) {
        distance_along_curve.clear();
    }

    // Find the closest point on the path and calculate cumulative distances
    for (size_t point_index = 0; point_index < path.size(); ++point_index) {
        const common_msgs::msg::PointXY& current_point = path[point_index];  
        double dx = current_point.x - car_position.x;
        double dy = current_point.y - car_position.y;
        double current_distance = dx * dx + dy * dy;

        // Update the closest point index
        if (current_distance < min_distance) {
            min_distance = current_distance;
            closest_point_index = point_index;
        }

        // Calculate the distance along the curve only if the path was updated
        if (path_updated) {
            if (point_index == 0) {
                distance_along_curve.push_back(0.0);
            } else {
                const common_msgs::msg::PointXY& previous_point = path[point_index - 1];  
                double dist_dx = current_point.x - previous_point.x;
                double dist_dy = current_point.y - previous_point.y;
                double distance_from_previous = std::sqrt(dist_dx * dist_dx + dist_dy * dist_dy);
                distance_along_curve.push_back(distance_from_previous + distance_along_curve.back());
            }
        }
    }

    // Find the pursuit point at the look-ahead distance
    double closest_point_dist = distance_along_curve[closest_point_index];
    for (size_t point_index = closest_point_index; point_index < path.size(); ++point_index) {
        double cur_dist = distance_along_curve[point_index] - closest_point_dist;
        if (cur_dist > look_ahead_distance) {
            pursuit_index = point_index;
            return path[pursuit_index];
        }
    }

    return path.back(); // If no point found, return the last point in the path
}

// Implementation of get_steering_angle
double PurePursuit::get_steering_angle() {
    if (path.size() <= 1) {
        return prev_steer; // If the path is too short, return the previous steering angle
    }

    // Variables for car's orientation (roll, pitch, yaw)
    double roll, pitch, yaw;

    if (global_mode) {
        geometry_msgs::msg::TransformStamped transform;
        try {
            // Get the transformation between the global frame and the car's frame
            transform = tf_buffer_.lookupTransform(global_frame, car_frame, tf2::TimePointZero);
            car_position.x = transform.transform.translation.x;
            car_position.y = transform.transform.translation.y;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
            return 0.0;
        }

        // Extract orientation from the transform
        tf2::Quaternion quaternion;
        tf2::fromMsg(transform.transform.rotation, quaternion);
        tf2::Matrix3x3 rotation_matrix(quaternion);
        rotation_matrix.getRPY(roll, pitch, yaw); // Get roll, pitch, and yaw from the quaternion
    } else {
        car_position.x = 0;
        car_position.y = 0;
        yaw = 0; // If in local mode, the car is at the origin with zero yaw
    }

    // Find the pursuit point
    common_msgs::msg::PointXY pursuit_point = search_pursuit_point(look_ahead_distance, car_position);

    // Calculate the steering angle based on the car's position and the pursuit point
    double alpha = std::atan2(pursuit_point.y - car_position.y, pursuit_point.x - car_position.x) - yaw;
    double delta = std::atan2(2.0 * 1.535 * std::sin(alpha) / look_ahead_distance, 1.0);

    // Clamp the steering angle to a max of +/- 19.9 degrees
    delta = std::max(-19.9, std::min(180 * delta / M_PI, 19.9));
    prev_steer = delta; // Store the steering angle for future use

    return delta;
}
