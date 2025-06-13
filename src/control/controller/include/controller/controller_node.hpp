/**
 * @file controller_node.hpp
 * @author Francis Rojas (frarojram@gmail.com).
 * @brief Controller node header for ARUS Team Driverless pipeline.
 */

#pragma once

#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/cmd.hpp"
#include "controller/pure_pursuit.hpp"
#include "controller/speed_control.hpp"
#include "controller/PID.hpp"
#include "controller/lti_mpc.hpp"
#include "controller/utils.hpp"


/**
 * @brief The Controller class
 * 
 * @details Create the controller object.
 */

class Controller : public rclcpp::Node
{
public:

    /**
     * @brief Constructor for the Controller class
     */
    Controller();
    
private:

    // Instances
    SpeedControl speed_control_;
    Pure_pursuit pure_pursuit_;   
    LtiMpc lti_mpc_;

    // State variables
    double x_;
    double y_;
    double yaw_;
    double vx_;
    double vy_;
    double r_;
    double delta_;

    double v_delta_{0.0};
    double prev_delta_{0.0};
    double target_speed_;

    double acc_cmd_;
    double delta_cmd_{0.0};
    bool run_check_ = false;
    bool optimized_ = false;

    //Trajectory variables
    std::vector<Point> pointsXY_;  
    std::vector<float> s_;                                         
    std::vector<float> k_ ; 
    std::vector<float> speed_profile_;               
    std::vector<float> acc_profile_;
    int index_global_ = 0;      
    bool new_trajectory_ = false;  
    bool braking_procedure_ = false;       
    

    // Topics
    std::string kStateTopic;
    std::string kRunCheckTopic;
    std::string kTrajectoryTopic;
    std::string kOptimizedTrajectoryTopic;
    std::string kCmdTopic;
    std::string kPursuitPointTopic;
    std::string kTargetSpeedTopic;
    std::string kBrakingProcedureTopic;

    // Parameters
    bool kUseOptimizedTrajectory;
    std::string kFirstLapSteerControl;
    std::string kOptimizedSteerControl;
    double kSpeedTimerFreq;
    double kSteerTimerFreq;
    double kLAD;
    double kOptLAD;
    double kTargetSpeed;
    double KP;
    double KI;
    double KD;
    double kMinCmd;
    double kMaxCmd;
    double kBrakingDecc;
    double kMaxSteer;
    double kCostLateralDeviation;
    double kCostAngularDeviation;
    double kCostSteeringDelta;
    int kCompensationSteps;
    double kWheelBase;
    double kWeightDistributionRear;
    double kRho;
    double kCdA;
    double kCrr;
    double kMass;
    double kIzz;
    double kG;
    bool kDebug;
    int kPredictionHorizon;
    double kTsMPC;
    double kCorneringStiffnessF;
    double kCorneringStiffnessR;
    double kSteerModelU;
    double kSteerModelDelta;
    double kSteerModelDeltaV;
    
    //Timers
    rclcpp::TimerBase::SharedPtr speed_timer_;
    rclcpp::TimerBase::SharedPtr steer_timer_;
    rclcpp::Time previous_time_ ;

    //Subscribers
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
    rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr trayectory_sub_;
    rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr optimized_trajectory_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_check_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr braking_procedure_sub_;

    // Publishers
    rclcpp::Publisher<common_msgs::msg::Cmd>::SharedPtr cmd_pub_;
    rclcpp::Publisher<common_msgs::msg::PointXY>::SharedPtr pursuit_point_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_speed_pub_;


    /**
     * @brief Speed control timer callback.
     */
    void on_speed_timer();

    /**
     * @brief Steering control timer callback.
     */
    void on_steer_timer();

    /**
     * @brief Car state callback.
     */
    void car_state_callback(const common_msgs::msg::State::SharedPtr msg);

    /**
     * @brief Trajectory callback. 
     * Subscribe to the trajectory topic and update the trajectory.
     */
    void trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);

    /**
     * @brief Optimized trajectory callback. 
     * Subscribe to the optimized trajectory topic and trigger optimized control mode.
     */
    void optimized_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg);

    /**
     * @brief Run check callback. 
     * Checks whether the vehicle is able to run or not.
     */
    void run_check_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Braking procedure callback. 
     * Subscribe to the braking procedure topic and trigger braking procedure.
     */
    void braking_procedure_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Get the global index of the vehicle in the trajectory.
     * Use the global position to calculate the speed and acceleration profile at each moment.
     */
    void get_global_index();

};
