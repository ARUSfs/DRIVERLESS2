/**
 * @file pure_pursuit.hpp
 * @author Francis Rojas (frarojram@gmail.com).
 * @brief Pure Pursuit, header and implementation for ARUS Team Driverless pipeline.
 */

#include <limits>
#include <cmath>
#include "controller/utils.hpp"


class Pure_pursuit
{
    public:
        Point pursuit_point_;
        double delta_cmd_;
        double wheel_base_;

        Pure_pursuit()
        {
            x_ = 0.0;
            y_ = 0.0;
            yaw_ = 0.0;
            delta_cmd_ = 0.0;
            pursuit_point_ = {0.0, 0.0};
            pursuit_index_ = 0;
            path_.clear();
        }

        /**
         * @brief Update the current trayectory.
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

        /**
         * @brief Set the current position of the car.
         */
        void set_position(double x, double y, double yaw)
        {
            x_ = x;
            y_ = y;
            yaw_ = yaw;
        }

        /**
         * @brief Search pursuit point. 
         * Returns the point on the trajectory closest to the look-ahead distance.
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
         * @brief Get steering angle to the pursuit point.
         */
        void get_steering_angle(int index_global, double LAD)
        {
            if (path_.size() <= 1)
            {
                return;
            }

            search_pursuit_point(index_global, LAD);

            double alpha = std::atan2(pursuit_point_.y - y_, pursuit_point_.x - x_) - yaw_;
            delta_cmd_ = std::atan2(2.0 * wheel_base_ * std::sin(alpha) / LAD, 1.0);
        }

    private:
        std::vector<Point> path_;

        double x_;
        double y_;
        double yaw_;

        size_t pursuit_index_;
};
