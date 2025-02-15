#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "graph_slam/landmark.hpp"

class DataAssociation{
    public:
        std::vector<Landmark*> map_;
        
        DataAssociation()
        {
            map_.clear();
        }

        void match_observations(std::vector<Landmark>& observed_landmarks, std::vector<Landmark>& unmatched_landmarks)
        {
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