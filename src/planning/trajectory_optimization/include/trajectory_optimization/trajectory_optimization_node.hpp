/**
 * @file trajectory_optimization_node.hpp
 * @author José Manuel Landero Plaza (josemlandero05@gmail.com)
 * @brief Trajectory Optimization node header for ARUS Team Driverless pipeline
 * @date 6-11-2024
 */
#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/state.hpp"
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
        //Car state variables
        double vx_;
        double vy_;
        double speed_;
        double ax_;
        double ay_;
        double acc_;

        //Parameters
        double kAxMax;
        double kAyMax;
        double kVMax;
        double kMinDist;
        double kMuY = 1.1;
        double kMuXThrottle = 0.6;
        double kMuxBrake = 0.9;
        double kG = 9.81;
        double kCLift = 0.5*1.2*3.5;
        double kCDrag = 0.5*1.2*1.2;
        double kMass = 270; 

        // Tracklimits
        std::vector<common_msgs::msg::PointXY> track_limit_right_;
        std::vector<common_msgs::msg::PointXY> track_limit_left_;

        std::string kTrajectoryTopic;
        std::string kCarStateTopic;
        std::string kOptimizedTrajectoryTopic;
        std::string kTrackLimitsTopic;

        //Subscribers and publishers
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
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
        void trajectory_callback(common_msgs::msg::Trajectory::SharedPtr trajectory_msg);
      
        /**
         * @brief Callback function for the car_state topic
         * 
         * We extract vx, vy, ax, ay to calculate the car's current speed and acceleration
         * 
         * @param car_state_msg 
         */        
        void car_state_callback(common_msgs::msg::State::SharedPtr car_state_msg);

        void track_limit_callback(common_msgs::msg::TrackLimits::SharedPtr track_limits_msg);


        //Auxiliar methods

        /**
         * @brief Generate track width limits on each point based on the trajectory's 
         * curvature to control the optimized trajectory's boundaries
         * 
         * @param  k Curvature of the given trajectory
         * @param  dmax Maximum track width
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
         * @return MatrixXd Matrix containing s and k: [s, k]
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