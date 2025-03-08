#pragma once
#define PCL_NO_PRECOMPILE
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "graph_slam/landmark.hpp"
#include "ConeXYZColorScore.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>


class DataAssociation{
    public:
        std::vector<Landmark*> map_;
        
        DataAssociation()
        {
            map_.clear();
        }

        void match_observations(std::vector<Landmark>& observed_landmarks, std::vector<Landmark>& unmatched_landmarks)
        {
            pcl::IterativeClosestPoint<ConeXYZColorScore, ConeXYZColorScore> icp;
            pcl::PointCloud<ConeXYZColorScore>::Ptr obs_map = pcl::PointCloud<ConeXYZColorScore>::Ptr(new pcl::PointCloud<ConeXYZColorScore>);

            for (Landmark& obs : observed_landmarks){
                ConeXYZColorScore point;
                point.x = obs.world_position_.x();
                point.y = obs.world_position_.y();
                point.z = 0;
                point.color = 0;
                point.score = 1;
                obs_map->push_back(point);
            }

            pcl::PointCloud<ConeXYZColorScore>::Ptr map = pcl::PointCloud<ConeXYZColorScore>::Ptr(new pcl::PointCloud<ConeXYZColorScore>);
 
            for (Landmark* landmark : map_){
                ConeXYZColorScore point;
                point.x = landmark->world_position_.x();
                point.y = landmark->world_position_.y();
                point.z = 0;
                point.color = 0;
                point.score = 1;
                map->push_back(point);
            }

            icp.setInputSource(obs_map);
            icp.setInputTarget(map);
            icp.setMaximumIterations(5);
            icp.setEuclideanFitnessEpsilon(0.005);
            icp.setTransformationEpsilon(1e-5);
 
            pcl::PointCloud<ConeXYZColorScore>::Ptr corrected_obs = pcl::PointCloud<ConeXYZColorScore>::Ptr(new pcl::PointCloud<ConeXYZColorScore>);
            icp.setMaxCorrespondenceDistance (1.0);
            icp.align(*corrected_obs);
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
 
            for (Landmark& obs : observed_landmarks){
                Eigen::Vector2d corrected_position;
                corrected_position << transformation(0,0)*obs.world_position_.x() + transformation(0,1)*obs.world_position_.y() + transformation(0,3),
                                      transformation(1,0)*obs.world_position_.x() + transformation(1,1)*obs.world_position_.y() + transformation(1,3);
                obs.world_position_ = corrected_position;
            }


            for(Landmark& obs : observed_landmarks){
                double min_distance = std::numeric_limits<double>::max();
                Landmark* closest_landmark = nullptr;
                for(Landmark* landmark : map_){
                    double distance = (obs.world_position_ - landmark->world_position_).norm();
                    if(distance < min_distance && !landmark->disabled_){
                        min_distance = distance;
                        closest_landmark = landmark;
                    }
                }
                if(min_distance < 1 && closest_landmark != nullptr){
                    obs.id_ = closest_landmark->id_;
                    obs.covariance_ = closest_landmark->covariance_;
                    closest_landmark->num_observations_++;
                    closest_landmark->last_observation_time_ = time(0);
                }
                else{
                    obs.id_ = Landmark::UNMATCHED_ID;
                    unmatched_landmarks.emplace_back(obs);
                }
            }

            // Disable landmarks that might be false positives
            for (Landmark* landmark : map_){
                if (time(0) - landmark->last_observation_time_ > 1.0 && landmark->num_observations_ < 10){
                    landmark->disabled_ = true;
                }
            }

            observed_landmarks.erase(std::remove_if(observed_landmarks.begin(), 
                                    observed_landmarks.end(), 
                                    [](Landmark& obs){return obs.id_ == Landmark::UNMATCHED_ID;}), 
                                    observed_landmarks.end());
        }

    
    private:
};
