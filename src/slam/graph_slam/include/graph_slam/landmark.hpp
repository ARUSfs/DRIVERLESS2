/**
 * @file landmark.hpp
 * @author Álvaro Landero (alplepe02@gmail.com)
 * @brief Landmark class for GraphSlam
 */

#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

// Landmark color definitions
#define UNCOLORED -1
#define BLUE 0
#define YELLOW 1
#define ORANGE 2
#define ORANGE_BIG 3


class Landmark {
    public:
        constexpr static int UNINITIALIZED_ID = -1;
        constexpr static int UNMATCHED_ID     = -2;

        int id_;            
        Eigen::Vector2d local_position_;    // Local position in the vehicle frame
        Eigen::Vector2d world_position_;    // World position in the map frame
        Eigen::Matrix2d covariance_;        // Covariance of the landmark position
        int color_;                         // UNCOLORED, BLUE, YELLOW, ORANGE, ORANGE_BIG         
        int num_observations_;
        time_t last_observation_time_;
        bool disabled_;

        /**
         * @brief Default constructor for Landmark. Initialized as UNCOLORED at origin
         */
        Landmark() {
            id_ = UNINITIALIZED_ID;
            local_position_ = Eigen::Vector2d::Zero();
            world_position_ = Eigen::Vector2d::Zero();
            covariance_ = Eigen::Matrix2d::Identity();
            color_ = UNCOLORED;
            disabled_ = false;
            last_observation_time_ = time(0);  
        }

        /**
         * @brief Constructor for Landmark with given world position
         */
        Landmark(const Eigen::Vector2d& world_position) {
            id_ = UNMATCHED_ID;
            world_position_ = world_position;
            covariance_ = Eigen::Matrix2d::Identity();
            color_ = UNCOLORED;
            num_observations_ = 0;
            disabled_ = false;
            last_observation_time_ = time(0);  
        }

        /**
         * @brief Constructor for Landmark with given local position and vehicle pose
         */
        Landmark(const Eigen::Vector2d& local_position, const Eigen::Vector3d vehicle_pose) {
            id_ = UNMATCHED_ID;
            local_position_ = local_position;
            get_world_pos(vehicle_pose);
            covariance_ = Eigen::Matrix2d::Identity();
            color_ = UNCOLORED;
            num_observations_ = 0;
            disabled_ = false;
            last_observation_time_ = time(0);  
        }

    private:
        /**
         * @brief Get the world position of the landmark from the local position and vehicle pose
         */
        void get_world_pos(const Eigen::Vector3d vehicle_pose) {
            const double phi = vehicle_pose(2);
            const double sinphi = std::sin(phi);
            const double cosphi = std::cos(phi);
            const Eigen::Matrix2d R = (Eigen::Matrix2d() << cosphi, -sinphi, sinphi, cosphi).finished();

            world_position_ =  R * local_position_ + vehicle_pose.head(2);
        }

};
