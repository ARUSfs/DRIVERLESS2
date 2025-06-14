/**
 * @file data_association_ICP_NN.hpp
 * @author Álvaro Landero (alplepe02@gmail.com)
 * @brief Nearest Neighbor Data Association of Landmarks with ICP
 */

#pragma once
#define PCL_NO_PRECOMPILE
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "graph_slam/landmark.hpp"
#include "ConeXYZColorScore.h"
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/kdtree/kdtree_flann.h>


class DataAssociation{
    public:
        std::vector<std::shared_ptr<Landmark>> map_;
        rclcpp::Logger logger_ = rclcpp::get_logger("DataAssociation");
        bool debug_ = true;
        
        /**
         * @brief Default constructor for DataAssociation. Initializes the map and ICP parameters
         */
        DataAssociation()
        {
            map_.clear();

            icp_.setMaximumIterations(50);
            icp_.setEuclideanFitnessEpsilon(0.1); 
            icp_.setTransformationEpsilon(1e-4); 
            icp_.setMaxCorrespondenceDistance(2.0);
        }

        /**
         * @brief Match given observations to the map using ICP and nearest neighbor search. 
         * Unmatched landmarks are returned in the unmatched_landmarks vector.
         */
        void match_observations(std::vector<Landmark>& observed_landmarks, std::vector<Landmark>& unmatched_landmarks)
        {
            // Perform ICP to match observed landmarks to map
            pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pcl = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr map_pcl = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            for (Landmark& obs : observed_landmarks){
                obs_pcl->push_back(pcl::PointXYZ(obs.world_position_.x(), obs.world_position_.y(), 0));
            }
            for (auto landmark : map_){
                map_pcl->push_back(pcl::PointXYZ(landmark->world_position_.x(), landmark->world_position_.y(), 0));
            }


            // Merge observations that are too close together
            int i = 0;
            while (i < obs_pcl->size()) {
                pcl::PointXYZ obs_point = obs_pcl->points[i];

                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                kdtree.setInputCloud(obs_pcl);
                std::vector<int> indices(2);
                std::vector<float> distances(2);

                if (kdtree.nearestKSearch(obs_pcl->points[i], 2, indices, distances) > 0 && distances[1] < 1.0) {
                    // Found a match
                    obs_pcl->points[i].x = (obs_pcl->points[i].x + obs_pcl->points[indices[1]].x)/2;
                    obs_pcl->points[i].y = (obs_pcl->points[i].y + obs_pcl->points[indices[1]].y)/2;
                    obs_pcl->points[i].z = (obs_pcl->points[i].z + obs_pcl->points[indices[1]].z)/2;

                    observed_landmarks[i].world_position_ = Eigen::Vector2d(obs_pcl->points[i].x, obs_pcl->points[i].y);
                    
                    obs_pcl->erase(obs_pcl->points.begin() + indices[1]); // Remove the second point
                    observed_landmarks.erase(observed_landmarks.begin() + indices[1]); // Remove the corresponding landmark
                    std::cout << "Merged points at index " << i << " and " << indices[1] << std::endl;
                }
                i++;
            }



            if (map_pcl->size() == 0) {
                for(Landmark& obs : observed_landmarks){
                    obs.id_ = Landmark::UNMATCHED_ID;
                    unmatched_landmarks.emplace_back(obs);
                }
                observed_landmarks.clear();
                return;
            } 
                    

            // Perform ICP to align the observed landmarks with the map
            icp_.setInputSource(obs_pcl);
            icp_.setInputTarget(map_pcl);
            pcl::PointCloud<pcl::PointXYZ>::Ptr corrected_obs = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            icp_.align(*corrected_obs);
            
            if (icp_.hasConverged() && icp_.getFitnessScore() < 1){
                for (int i = 0; i < observed_landmarks.size(); i++){
                    Landmark& obs = observed_landmarks[i];
                    obs.world_position_ = Eigen::Vector2d(corrected_obs->points[i].x, corrected_obs->points[i].y);
                }
            } else {
                if (debug_) RCLCPP_WARN(this->logger_, "ICP did not converge. Score: %f", icp_.getFitnessScore());
            }


            // Get nearest neighbor for each observed landmark
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(map_pcl);

            for (int i = 0; i < corrected_obs->size(); i++) {
                pcl::PointXYZ obs_point = corrected_obs->points[i];
                std::vector<int> indices(1);
                std::vector<float> distances(1);

                if (kdtree.nearestKSearch(obs_point, 1, indices, distances) > 0 && distances[0] < 1.0) {
                    // Found a match
                    Landmark& obs = observed_landmarks[i];
                    auto closest_landmark = map_[indices[0]];
                    obs.id_ = closest_landmark->id_;
                    obs.covariance_ = closest_landmark->covariance_;
                    closest_landmark->num_observations_++;
                    closest_landmark->last_observation_time_ = time(0);
                    closest_landmark->update_color(obs.prob_blue_, obs.prob_yellow_);
                } else {
                    // No match found, mark as unmatched
                    Landmark& obs = observed_landmarks[i];
                    obs.id_ = Landmark::UNMATCHED_ID;
                    unmatched_landmarks.emplace_back(obs);
                }
                    
            }
            
            





            // Disable landmarks that might be false positives
            for (auto landmark : map_){
                if (time(0) - landmark->last_observation_time_ > 1.0 && landmark->num_observations_ < 3){
                    // landmark->disabled_ = true;
                }
            }

            observed_landmarks.erase(std::remove_if(observed_landmarks.begin(), 
                                    observed_landmarks.end(), 
                                    [](Landmark& obs){return obs.id_ == Landmark::UNMATCHED_ID;}), 
                                    observed_landmarks.end());
        }

    
    private:
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
};
