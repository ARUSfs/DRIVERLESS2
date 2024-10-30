#include <common_msgs/msg/point_xy.hpp>
#include <vector>
#include <limits>
#include <cmath>

std::vector<common_msgs::msg::PointXY> path;  
bool path_updated = false; 
common_msgs::msg::PointXY car_position;
std::vector<double> distance_along_curve; 
size_t pursuit_index;
double prev_steer;

namespace PurePursuit {

    void set_path(const std::vector<common_msgs::msg::PointXY> &new_path) {
        if (new_path.empty()) {
            return;
        }
        path = new_path;
        path_updated = true;
    }

    void set_position(const common_msgs::msg::PointXY &position) {
        car_position = position;
    }

    common_msgs::msg::PointXY search_pursuit_point(double look_ahead_distance) {

        size_t closest_point_index = 0;
        double min_distance = std::numeric_limits<double>::infinity();

        if (path_updated) {
            distance_along_curve.clear();
        }

        for (size_t point_index = 0; point_index < path.size(); ++point_index) {
            const common_msgs::msg::PointXY &current_point = path[point_index];
            double dx = current_point.x - car_position.x;
            double dy = current_point.y - car_position.y;
            double current_distance = dx * dx + dy * dy;

            if (current_distance < min_distance) {
                min_distance = current_distance;
                closest_point_index = point_index;
            }

            if (path_updated) {
                if (point_index == 0) {
                    distance_along_curve.push_back(0.0);
                } else {
                    const common_msgs::msg::PointXY &previous_point = path[point_index - 1];
                    double dist_dx = current_point.x - previous_point.x;
                    double dist_dy = current_point.y - previous_point.y;
                    double distance_from_previous = std::sqrt(dist_dx * dist_dx + dist_dy * dist_dy);
                    distance_along_curve.push_back(distance_from_previous + distance_along_curve.back());
                }
            }
        }

        double closest_point_dist = distance_along_curve[closest_point_index];
        for (size_t point_index = closest_point_index; point_index < path.size(); ++point_index) {
            double cur_dist = distance_along_curve[point_index] - closest_point_dist;
            if (cur_dist > look_ahead_distance) {
                pursuit_index = point_index;
                return path[pursuit_index];
            }
        }

        return path.back(); 
    }

    double get_steering_angle(double look_ahead_distance) {
        if (path.size() <= 1) {
            return prev_steer;
        }

        common_msgs::msg::PointXY pursuit_point = search_pursuit_point(look_ahead_distance);

        double alpha = std::atan2(pursuit_point.y - car_position.y, pursuit_point.x - car_position.x);
        double delta = std::atan2(2.0 * 1.535 * std::sin(alpha) / look_ahead_distance, 1.0);

        delta = std::max(-0.347, std::min(delta, 0.347));
        prev_steer = delta;

        return delta;
    }

}