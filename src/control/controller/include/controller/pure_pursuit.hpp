/**
 * @file pure_pursuit.hpp
 *
 * @author Francis Rojas (frarojram@gmail.com).
 *
 * @brief Pure Pursuit, header and implementation for ARUS Team Driverless pipeline.
 *
 * @date 20-11-2024
 */

#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include "Point.h"

class Pure_pursuit
{
public:
    Point pursuit_point_;
    double delta_cmd_;

    Pure_pursuit()
    {
        car_position_ = {0.0, 0.0};
        delta_cmd_ = 0.0;
        pursuit_point_ = {0.0, 0.0};
        pursuit_index_ = 0;
        path_.clear();
    }

    /**
     * @brief Set path trayectory, set global position and yaw
     * @param new_path
     * @param position
     * @param yaw
     * @details Update the current trayectory.
     */
    void set_path(const std::vector<Point> &new_path)
    {
        if (new_path.empty())
        {
            return;
        }
        path_.clear();
        path_ = new_path;
    }

    void set_position(const Point &position, double yaw)
    {
        car_position_ = position;
        yaw_ = yaw;
    }
    /**
     * @brief Search pursuit point.
     * @author Francis Rojas (frarojram@gmail.com).
     * @param index_global
     * @param look_ahead_distance
     * @details Through a position, it takes the point on the trajectory that matches
     *          the look-ahead distances and returns the candidate.
     * @return Point to pursue.
     */
    void search_pursuit_point(size_t index_global, double look_ahead_distance)
    {
        double accumulated_distance = 0.0;
        int N = path_.size();
        int point_index = index_global;

        while (look_ahead_distance >= accumulated_distance)
        {
            const Point &current_point = path_[point_index % N];
            const Point &next_point = path_[(point_index + 1) % N];

            double dx = next_point.x - current_point.x;
            double dy = next_point.y - current_point.y;

            double segment_distance = std::sqrt(dx * dx + dy * dy);

            accumulated_distance += segment_distance;

            if (accumulated_distance >= look_ahead_distance)
            {
                pursuit_index_ = (point_index + 1) % N;
                pursuit_point_ = path_[pursuit_index_];
                return;
            }
            ++point_index;
        }

        pursuit_point_ = path_.back();
    }

    /**
     * @brief Get cross track error.
     * @authors Lola Hernández (lolahercan@gmail.com)
     * @param car_position
     * @param path
     * @param global index
     * @details Calculates the cross track error, difference between car position and path
     * @return ye, cross track error
     */

    double get_cross_track_error(const std::vector<Point> &trajectory, const Point &position)
    {
        if (trajectory.empty()) return 0.0;

        double min_dist = std::numeric_limits<double>::max();
        Point closest_point;

        for (const auto& pt : trajectory){
            double dist = std::hypot(pt.x - position.x, pt.y - position.y);
            if(dist < min_dist){
                min_dist = dist;
                closest_point = pt;
            }
        }
        return min_dist;
    }

    /**
     * @brief Get look ahead distance.
     * @authors Lola Hernández (lolahercan@gmail.com)
     * @param cross track error
     * @details Calculate the look ahead distance using formula and fixed parameters
     * @return look ahead distance 
     */

     double calculate_look_ahead_distance(double cross_track_error, double vx) {
        double delta_min_ = 5.5;  
        double delta_max_ = 12.0; 
        double gamma_ = 1.0;        
        
        double vel_f = std::min(std::max(vx / 10.0, 0.5), 2.0);

        return vel_f * ((delta_max_ - delta_min_) * std::exp(-gamma_ * std::abs(cross_track_error)) + delta_min_);
   
    }
    
    /**
     * @brief Get steering angle.
     * @authors Team driverless ARUS
     * @param index_global
     * @param look_ahead_distance
     * @details From the pursuit point, it calculates the steering
     *          angle to send in the command.
     * @return Point to pursue.
     */
    void get_steering_angle(int index_global, double cross_track_error, double vx)
    {
        if (path_.size() <= 1)
        {
            return;
        }

        double look_ahead_distance = calculate_look_ahead_distance(cross_track_error, vx);
        std::cout << "Look Ahead Distance: " << look_ahead_distance << std::endl;

        search_pursuit_point(index_global, look_ahead_distance);

        double alpha = std::atan2(pursuit_point_.y - car_position_.y, pursuit_point_.x - car_position_.x) - yaw_;
        delta_cmd_ = std::atan2(2.0 * wheel_base_ * std::sin(alpha) / look_ahead_distance, 1.0);
        
    }

private:
    std::vector<Point> path_;

    Point car_position_;
    double yaw_;

    size_t pursuit_index_;

    double wheel_base_ = 1.535; // meters
};
