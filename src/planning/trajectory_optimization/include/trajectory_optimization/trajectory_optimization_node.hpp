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
        // Tracklimits variables
        std::vector<common_msgs::msg::PointXY> track_limit_right_;
        std::vector<common_msgs::msg::PointXY> track_limit_left_;

        // Parameters
        // Optimized trajectory parameters
        int kNIter;
        int kNSeg;
        double kMinDist;

        // Profile parameters
        double kVMax;
        double kMuY;
        double kMuXThrottle;
        double kMuxBrake;

        // Car parameters
        double kG ;
        double kMass; 
        double kCLift;
        double kCDrag;

        // Message topics
        std::string kOptimizedTrajectoryTopic;
        std::string kTrackLimitsTopic;

        // Debug mode
        bool kDebug;

        // Subscribers
        rclcpp::Subscription<common_msgs::msg::TrackLimits>::SharedPtr track_limits_sub_;

        // Publishers
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr optimized_trajectory_pub_;

        // Callbacks
        /**
         * @brief Callback function for the track limits topic. It receives the midpath and tracklimits 
         * and calculates an optimized trajectory, minimizing curvature.
         */
        void trajectory_callback(common_msgs::msg::TrackLimits::SharedPtr trajectory_msg);

        // Auxiliar functions
        /**
         * @brief Generates optimized trajectory track width allowed at each midpoint based on the points' 
         * distance to the track limits.
         */
        VectorXd generate_track_width(VectorXd x, VectorXd y, std::vector<common_msgs::msg::PointXY> track_limit);

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

        /**
         * @brief Creates the trajectory message to publish.
         */
        common_msgs::msg::Trajectory create_trajectory_msg(VectorXd traj_x, VectorXd traj_y, 
            VectorXd s, VectorXd k, VectorXd speed_profile, VectorXd acc_profile);
};