/**
 * @file pure_pursuit.hpp
 * 
 * @author Francis Rojas (frarojram@gmail.com).
 * 
 * @brief Pure Pursuit implementation for ARUS Team Driverless pipeline.
 * 
 * @date 15-11-2024
 */

#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include "Point.h"

std::vector<Point> path;  
bool path_updated = false; 
std::vector<double> distance_along_curve; 
size_t pursuit_index;
double prev_steer = 0.0;
Point car_position;
double yaw;


namespace PurePursuit {
    /**
     * @brief Set path trayectory.
     * 
     * @param new_path
     * 
     * @details Update the current trayectory.
     */
    void set_path(const std::vector<Point> &new_path) {
        if (new_path.empty()) {
            return;
        }
        path = new_path;
        path_updated = true;
    }

    /**
     * @brief Set global position.
     * 
     * @authors Team driverless ARUS
     * 
     * @param position
     * 
     * @param yaw_
     * 
     * @details Update the current position.
     */
    void set_position(const Point &position, double yaw_) {
        car_position = position;
        yaw = yaw_;
    }

    /**
     * @brief Search pursuit point.
     * 
     * @authors Team driverless ARUS
     * 
     * @param look_ahead_distance
     * 
     * @details Through a position, it takes the point on the trajectory that matches
     *          the look-ahead distances and returns the candidate.
     * 
     * @return Point to pursue.
     */
    Point search_pursuit_point(double look_ahead_distance) {

        size_t closest_point_index = 0;
        double min_distance = std::numeric_limits<double>::infinity();

        if (path_updated) {
            distance_along_curve.clear();
        }

        for (size_t point_index = 0; point_index < path.size(); ++point_index) {
            const Point &current_point = path[point_index];
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
                    const Point &previous_point = path[point_index - 1];
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
    /**
     * @brief Get steering angle.
     * 
     * @authors Team driverless ARUS
     * 
     * @param look_ahead_distance
     * 
     * @details From the pursuit point, it calculates the steering 
     *          angle to send in the command.
     * 
     * @return Point to pursue.
     */
    double get_steering_angle(double look_ahead_distance) {
        if (path.size() <= 1) {
            return prev_steer;
        }

        Point pursuit_point = search_pursuit_point(look_ahead_distance);

        double alpha = std::atan2(pursuit_point.y - car_position.y, pursuit_point.x - car_position.x) - yaw;
        double delta = std::atan2(2.0 * 1.535 * std::sin(alpha) / look_ahead_distance, 1.0);

        delta = std::max(-0.347, std::min(delta, 0.347));
        delta = std::clamp(delta, -20*M_PI/180, 20*M_PI/180);
        prev_steer = delta;

        return delta;
    }

}
