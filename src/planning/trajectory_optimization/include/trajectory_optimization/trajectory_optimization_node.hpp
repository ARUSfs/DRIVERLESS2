/**
 * @file trajectory_optimization_node.hpp
 * @author José Manuel Landero Plaza (josemlandero05@gmail.com)
 * @brief Trajectory Optimization node header for ARUS Team Driverless pipeline
 */
#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/track_limits.hpp"
#include "libInterpolate/Interpolate.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
using namespace std;
using namespace Eigen;

/**
 * @class TrajectoryOptimization
 * @brief TrajectoryOptimization class
 * 
 * This class generates optimized trajectory (minimal curvature) based on the midpath and
 * track limits received from path planning
 * 
 */
class TrajectoryOptimization : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the TrajectoryOptimization node
         */
        TrajectoryOptimization();

    private:
        //Parameters
        double kVMax;
        double kMinDist;
        double kMuY;
        double kMuXThrottle;
        double kMuxBrake;
        double kG = 9.81;
        double kCLift = 0.5*1.2*3.5;
        double kCDrag = 0.5*1.2*1.2;
        double kMass = 270; 
        int kNIter;
        int kNSeg;

        // Tracklimits
        std::vector<common_msgs::msg::PointXY> track_limit_right_;
        std::vector<common_msgs::msg::PointXY> track_limit_left_;

        std::string kTrajectoryTopic;
        std::string kCarStateTopic;
        std::string kOptimizedTrajectoryTopic;
        std::string kTrackLimitsTopic;

        //Subscribers and publishers
        rclcpp::Subscription<common_msgs::msg::TrackLimits>::SharedPtr track_limits_sub_;
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr optimized_trajectory_pub_;

        /**
         * @brief Callback function for the track limits topic. It receives the midpath and tracklimits 
         * and calculates an optimized trajectory, minimizing curvature.
         */
        void trajectory_callback(common_msgs::msg::TrackLimits::SharedPtr trajectory_msg);

        /**
         * @brief Generates optimized trajectory track width allowed at each midpoint based on the points' 
         * distance to the track limits.
         */
        VectorXd generate_track_width(VectorXd x, VectorXd y, std::vector<common_msgs::msg::PointXY> track_limit);

        /**
         * @brief Creates the trajectory message to publish.
         */
        common_msgs::msg::Trajectory create_trajectory_msg(VectorXd traj_x, VectorXd traj_y, 
            VectorXd s, VectorXd k, VectorXd speed_profile, VectorXd acc_profile);

        /**
         * @brief Calculates the accumulated distance (s) and curvature (k) at each point of the given trajectory.
         */
        MatrixXd get_distance_and_curvature_values(VectorXd traj_x, VectorXd traj_y);

        /**
         * @brief Generates speed and acceleration profiles for the trajectory
         */
        MatrixXd generate_speed_and_acc_profile(VectorXd s, VectorXd k);

        /**
         * @brief Calculates maximum speed to maintain grip for a given curvature.
         */
        double calculate_apex(double k);
        
        /**
         * @brief Calculates maximum possible acceleration for given speed and curvature.
         */
        double ggv_ax_throttle(double v, double k);

        /**
         * @brief Calculates maximum braking acceleration for given speed and curvature.
         */
        double ggv_ax_brake(double v, double k);
};