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
            double tolerance = 1.0;
            int s = 0;
            do
            {
                s=0;
                for (Landmark& obs: observed_landmarks){
                    obs.id_ = Landmark::UNINITIALIZED_ID;
                }


                for(Landmark& obs : observed_landmarks){
                    double min_distance = std::numeric_limits<double>::max();
                    Landmark* closest_landmark = nullptr;
                    for(Landmark* landmark : map_){
                        double distance = (obs.world_position_ - landmark->world_position_).norm();
                        if(distance < min_distance){
                            min_distance = distance;
                            closest_landmark = landmark;
                        }
                    }
                    if(min_distance < tolerance && closest_landmark != nullptr){
                        obs.id_ = closest_landmark->id_;
                        obs.covariance_ = closest_landmark->covariance_;
                    }
                    else{
                        obs.id_ = Landmark::UNMATCHED_ID;
                        unmatched_landmarks.emplace_back(obs);
                        s++;
                    }
                }


                tolerance*=1.2;
                std::cout << "unmatched landmarks: " << s << std::endl;
                std::cout << "observed landmarks: " << observed_landmarks.size() << std::endl;
                std::cout << "tolerance: " << tolerance << std::endl;
                std::cout << "-----" << std::endl;
            } while ( s > observed_landmarks.size()/3 && map_.size() > 10);

            observed_landmarks.erase(std::remove_if(observed_landmarks.begin(), 
                                    observed_landmarks.end(), 
                                    [](Landmark& obs){return obs.id_ == Landmark::UNMATCHED_ID;}), 
                                    observed_landmarks.end());
        }

    
    private:
};