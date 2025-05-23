/**
 * @file trajectory_optimization_node.hpp
 * @author José Manuel Landero Plaza (josemlandero05@gmail.com)
 * @brief Trajectory Optimization node header for ARUS Team Driverless pipeline
 * @date 6-11-2024
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
 * This class generates optimized trajectory (minimal curvature) based on the trajectory received 
 * from path planning (right now, it takes trajectory from arussim_interface to allow testing)
 * 
 */
class TrajectoryOptimization : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the TrajectoryOptimization class
         * 
         * It initializes the Trajectory Optimization node, declaring all necessary parameters  
         * and creating the subscribers and publishers
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
         * @brief Callback function for the trajectory topic
         * 
         *  When a trajectory message is received, the callback extracts the track centerline 
         * (x,y) points from the message and executes all  necessary computations to get the 
         *  optimized trajectory full message 
         *  
         * @param trajectory_msg 
         */
        void trajectory_callback(common_msgs::msg::TrackLimits::SharedPtr trajectory_msg);

        //Auxiliar methods

        /**
         * @brief Generate optimized trajectory track width limits on each point based on the points' 
         * distance to the track limits
         * 
         * @param x Trajectory points' x coordenates vector
         * @param y Trajectory points' y coordenates vector
         * @param track_limit Track limits points XY vector
         * 
         * @return VectorXd Vector of track width allowed at each point (both left and rigth)
         */
        VectorXd generate_track_width(VectorXd x, VectorXd y, std::vector<common_msgs::msg::PointXY> track_limit);

        /**
         * @brief Creates the trajectory message to publish
         * 
         * @param  traj_x x coordinates of the trajectory points
         * @param  traj_y y coordinates of the trajectory points
         * @param  s Accumulated distance at each point
         * @param  k Curvature at each point
         * @param  speed_profile Speed profile for the given trajectory
         * @param  acc_profile Acceleration profile for the given trajectory
         * 
         * @return common_msgs::msg::Trajectory 
         */
        common_msgs::msg::Trajectory create_trajectory_msg(VectorXd traj_x, VectorXd traj_y, 
            VectorXd s, VectorXd k, VectorXd speed_profile, VectorXd acc_profile);

        /**
         * @brief Calculates the accumulated distance (s) and curvature (k) 
         * at each point of the given trajectory
         * 
         * @param  traj_x x coordinates of the given trajectory points
         * @param  traj_y y coordinates of the given trajectory points
         * 
         * @return MatrixXd [s, k]
         */
        MatrixXd get_distance_and_curvature_values(VectorXd traj_x, VectorXd traj_y);

        /**
         * @brief Generates speed and acceleration profiles for the trajectory
         * 
         * @param  s Accumulated distance at each point
         * @param  k Curvature at each point
         * 
         * @return MatrixXd [speed_profile, acc_profile]
         */
        MatrixXd generate_speed_and_acc_profile(VectorXd s, VectorXd k);

        double calculate_apex(double k);
        
        double ggv_ax_throttle(double v, double k);

        double ggv_ax_brake(double v, double k);
};