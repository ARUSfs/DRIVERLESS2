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
#include <unordered_set>
#include <limits>

struct Association {
    int observation_idx;
    int landmark_idx;
};

class DataAssociation {
public:
    std::vector<Landmark*> map_;

    DataAssociation(double chi2_threshold) : jcbb_(chi2_threshold) {
        map_.clear();
    }

    void match_observations(std::vector<Landmark>& observed_landmarks, std::vector<Landmark>& unmatched_landmarks) {
        // Convert landmarks and observations to Eigen::Vector2d for JCBB
        std::vector<Eigen::Vector2d> observations;
        std::vector<Eigen::Vector2d> landmarks;

        for (Landmark& obs : observed_landmarks) {
            observations.emplace_back(obs.world_position_);
        }

        for (Landmark* landmark : map_) {
            landmarks.emplace_back(landmark->world_position_);
        }

        // Perform JCBB matching
        std::vector<Association> associations = jcbb_.match(observations, landmarks);

        // Update observed landmarks with JCBB results
        std::unordered_set<int> matched_landmarks;
        for (const auto& assoc : associations) {
            Landmark& obs = observed_landmarks[assoc.observation_idx];
            Landmark* matched_landmark = map_[assoc.landmark_idx];

            obs.id_ = matched_landmark->id_;
            obs.covariance_ = matched_landmark->covariance_;
            matched_landmark->num_observations_++;
            matched_landmark->last_observation_time_ = time(0);

            matched_landmarks.insert(assoc.landmark_idx);
        }

        // Handle unmatched landmarks
        for (size_t i = 0; i < observed_landmarks.size(); ++i) {
            if (observed_landmarks[i].id_ == Landmark::UNMATCHED_ID) {
                unmatched_landmarks.emplace_back(observed_landmarks[i]);
            }
        }

        // Disable landmarks that might be false positives
        for (Landmark* landmark : map_) {
            if (time(0) - landmark->last_observation_time_ > 1.0 && landmark->num_observations_ < 3) {
                std::cout << "Desactivando landmark " << landmark->id_ 
                          << " por inactividad de " << (time(0) - landmark->last_observation_time_) 
                          << " s y solo " << landmark->num_observations_ << " observaciones" << std::endl;
                landmark->disabled_ = true;
            }
            
        }

        // Remove unmatched landmarks from observed_landmarks
        observed_landmarks.erase(std::remove_if(observed_landmarks.begin(),
                                                observed_landmarks.end(),
                                                [](Landmark& obs) { return obs.id_ == Landmark::UNMATCHED_ID; }),
                                 observed_landmarks.end());
    }

private:
    class JCBB {
    public:
        JCBB(double chi2_threshold) : chi2_threshold_(chi2_threshold) {}

        std::vector<Association> match(const std::vector<Eigen::Vector2d>& observations,
                                       const std::vector<Eigen::Vector2d>& landmarks) {
            std::vector<Association> best_associations;
            std::vector<Association> current_associations;
            std::unordered_set<int> used_landmarks;

            jcbb_recursive(best_associations, current_associations, observations, landmarks, 0, used_landmarks);
            return best_associations;
        }

    private:
        void jcbb_recursive(std::vector<Association>& best, std::vector<Association>& current,
                            const std::vector<Eigen::Vector2d>& observations,
                            const std::vector<Eigen::Vector2d>& landmarks,
                            int obs_idx, std::unordered_set<int>& used_landmarks) {
            if (obs_idx >= observations.size()) {
                if (current.size() > best.size()) {
                    best = current;
                }
                return;
            }

            bool found_association = false;
            for (int i = 0; i < landmarks.size(); ++i) {
                if (used_landmarks.find(i) != used_landmarks.end()) {
                    continue; // Skip already used landmarks
                }

                double d = mahalanobis_distance(observations[obs_idx], landmarks[i]);
                if (d < chi2_threshold_) {
                    current.push_back({obs_idx, i});
                    used_landmarks.insert(i);
                    jcbb_recursive(best, current, observations, landmarks, obs_idx + 1, used_landmarks);
                    used_landmarks.erase(i);
                    current.pop_back();
                    found_association = true;
                }
            }

            // If no association was found for the current observation, continue with the next observation
            if (!found_association) {
                jcbb_recursive(best, current, observations, landmarks, obs_idx + 1, used_landmarks);
            }
        }

        double mahalanobis_distance(const Eigen::Vector2d& obs, const Eigen::Vector2d& landmark) {
            Eigen::Vector2d diff = obs - landmark;
            Eigen::Matrix2d covariance;
            covariance << 0.1, 0.0,
            0.0, 0.1;  // Assuming identity covariance for simplicity
            return diff.dot(covariance.inverse() * diff);
        }

        double chi2_threshold_;
    };

    JCBB jcbb_;
};
