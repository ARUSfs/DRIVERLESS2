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
    Pure_pursuit()
    {
        car_position_ = {0.0, 0.0};
        prev_steer_ = 0.0;
        prev_pursuit_point_ = {0.0, 0.0};
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
    Point search_pursuit_point(size_t index_global, double look_ahead_distance)
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
                return path_[pursuit_index_];
            }
            ++point_index;
        }

        return path_.back();
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

    double calculateCrossTrackError(const Point &car_position,
                                    const std::vector<Point> &path,
                                    int global_index)
    {

        const Point &nearest_point = path[global_index];
        const Point &next_point = path[(global_index + 1) % path.size()];

        double alpha_k = std::atan2(next_point.y - nearest_point.y,
                                    next_point.x - nearest_point.x);

        double ye = -(car_position.x - nearest_point.x) * std::sin(alpha_k) +
                    (car_position.y - nearest_point.y) * std::cos(alpha_k);

        return ye;
    }

    /**
     * @brief Get look ahead distance.
     * @authors Lola Hernández (lolahercan@gmail.com)
     * @param cross track error
     * @details Calculate the look ahead distance using formula and fixed parameters
     * @return look ahead distance
     */

    double calculateLookAheadDistance(double cross_track_error)
    {
        double delta_min_ = 1.0;
        double delta_max_ = 8.0;
        double gamma_ = 1.0;

        return (delta_max_ - delta_min_) * std::exp(-gamma_ * std::abs(cross_track_error)) + delta_min_;
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
    std::pair<double, Point> get_steering_angle(int index_global, double cross_track_error)
    {
        if (path_.size() <= 1)
        {
            return {prev_steer_, prev_pursuit_point_};
        }

        double look_ahead_distance = calculateLookAheadDistance(cross_track_error);

        Point pursuit_point = search_pursuit_point(index_global, look_ahead_distance);

        double alpha = std::atan2(pursuit_point.y - car_position_.y, pursuit_point.x - car_position_.x) - yaw_;
        double delta = std::atan2(2.0 * 1.535 * std::sin(alpha) / look_ahead_distance, 1.0);

        delta = std::max(-0.347, std::min(delta, 0.347));
        delta = std::clamp(delta, -20 * M_PI / 180, 20 * M_PI / 180);

        prev_steer_ = delta;
        prev_pursuit_point_ = pursuit_point;

        return {delta, pursuit_point};
    }

private:
    std::vector<Point> path_;

    Point car_position_;
    double yaw_;

    size_t pursuit_index_;
    Point prev_pursuit_point_;
    double prev_steer_;
};
