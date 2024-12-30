#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>


class Landmark {
    public:
        constexpr static int UNINITIALIZED_ID = -1;
        constexpr static int UNMATCHED_ID     = -2;

        int id_;
        Eigen::Vector2d local_position_;
        Eigen::Vector2d world_position_;
        Eigen::Matrix2d covariance_;

        Landmark() {
            id_ = UNINITIALIZED_ID;
            local_position_ = Eigen::Vector2d::Zero();
            world_position_ = Eigen::Vector2d::Zero();
            covariance_ = Eigen::Matrix2d::Identity();
        }

        Landmark(const Eigen::Vector2d& world_position) {
            id_ = UNMATCHED_ID;
            world_position_ = world_position;
            covariance_ = Eigen::Matrix2d::Identity();
        }

        Landmark(const Eigen::Vector2d& local_position, const Eigen::Vector3d vehicle_pose) {
            id_ = UNMATCHED_ID;
            local_position_ = local_position;
            get_world_pos(vehicle_pose);
            covariance_ = Eigen::Matrix2d::Identity();
        }

    private:
        void get_world_pos(const Eigen::Vector3d vehicle_pose) {
            const double phi = vehicle_pose(2);
            const double sinphi = std::sin(phi);
            const double cosphi = std::cos(phi);
            const Eigen::Matrix2d R = (Eigen::Matrix2d() << cosphi, -sinphi, sinphi, cosphi).finished();

            world_position_ =  R * local_position_ + vehicle_pose.head(2);
        }

};