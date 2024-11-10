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

        //Parameters
        double kAxMax;
        double kAyMax;
        double kVMax;
        double kDMax;

        std::string kTrajectoryTopic;
        std::string kCarStateTopic;
        std::string kOptimizedTrajectoryTopic;

        //Subscribers and publishers
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
        rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr optimized_trajectory_pub_;

        /**
         * @brief Callback function for the trajectory topic
         * 
         *  When a trajectory message is received, the callback extracts the track centerline 
         * (x,y) points from the message and executes all  necessary computations to get the 
         *  optimized trajectory full message (except acceleration profile at this moment)
         *  
         * @param trajectory_msg 
         */
        void trajectory_callback(common_msgs::msg::Trajectory::SharedPtr trajectory_msg);
      
        /**
         * @brief Callback function for the car_state topic
         * 
         * We extract vx and vy to calculate the car's current speed
         * 
         * @param car_state_msg 
         */        
        void car_state_callback(common_msgs::msg::State::SharedPtr car_state_msg);


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
        VectorXd generate_track_width(VectorXd k, double dmax);

        /**
         * @brief Creates the trajectory message to publish
         * 
         * @param  traj_x x coordinates of the trajectory points
         * @param  traj_y y coordinates of the trajectory points
         * @param  s Accumulated distance at each point
         * @param  k Curvature at each point
         * @param  speed_profile Speed profile for the given trajectory
         * 
         * @return common_msgs::msg::Trajectory 
         */
        common_msgs::msg::Trajectory create_trajectory_msg(VectorXd traj_x, VectorXd traj_y, 
            VectorXd s, VectorXd k, VectorXd speed_profile);

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
         * @brief Generates a speed profile for the trajectory
         * 
         * @param  s Accumulated distance at each point
         * @param  k Curvature at each point
         * 
         * @return VectorXd Speed profile vector
         */
        VectorXd generate_speed_profile(VectorXd s, VectorXd k);
};