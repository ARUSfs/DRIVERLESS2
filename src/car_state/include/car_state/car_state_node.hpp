/**
 * @file car_state_node.hpp
 * @author Álvaro Landero (alplepe02@gmail.com)
 * @brief Car State node header for ARUS Team Driverless pipeline
 */

// ROS2
#include <rclcpp/rclcpp.hpp>

// Standard ROS 2 messages
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Custom messages
#include "common_msgs/msg/cmd.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/car_info.hpp"

// TF2 and geometry tools
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Internal utilities
#include "car_state/speed_estimator.hpp"



class CarState : public rclcpp::Node
{
public:
    /**
     * @brief Constructor of CarState node.
     */
    CarState();

private:
    // Variable to estimates the vehicle's longitudinal and lateral velocity.
    SpeedEstimator speed_estimator_;

    // Car variables
    double x_=0;
    double y_=0;
    double yaw_ = 0;
    double v_front_right_ = 0;
    double v_front_left_ = 0;
    double v_rear_right_ = 0;
    double v_rear_left_ = 0;
    double vx_ = 0;
    double vy_ = 0;
    double r_ = 0;
    double ax_ = 0;
    double ay_ = 0;
    double delta_ = 0;
    double delta_der_ = 0;
    double inv_speed_ = 0;

    int as_status_ = 0;
    int ami_ = 0;
    int ebs_status_ = 0;
    int ebs_redundancy_status_ = 0;
    double target_speed_ = -1;
    double target_delta_ = 0;
    int steering_state_ = 0;
    double torque_actual_ = 0;
    double torque_target_ = 0;
    double brake_hydr_pressure_ = 0;
    int lap_count_ = 0;
    int cones_count_actual_ = 0;
    int cones_count_all_ = 0;
    double plausability_ = 0;
    bool epos_OK_ = true; //TODO: subscribe from epos_interface

    // TF variables
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;


    // Parameters
    bool kSimulation;
    bool kGetArussimGroundTruth;

    std::string kMission;
    int kTrackdriveLaps;

    bool kSafeMode;
    bool kUseWheelspeeds;
    double kThresholdImu;
    double kThresholdExtensometer;
    double kThresholdWheelSpeed;
    double kThresholdInv;
    double kThresholdConesCountActual;
    double kThresholdConesCountAll;
    double kMaxError;

    double kMaxAx;
    double kMaxAy;
    double kMaxR;
    double kMaxVx;
    double kMaxPlausabilityError;

    double kErrorWeightIMU;
    double kErrorWeightExtensometer;
    double kErrorWeightWheelSpeed;
    double kErrorWeightInvSpeed;
    double kErrorWeightConesCountActual;
    double kErrorWeightConesCountAll;    

    bool kDebug;


    // Topics
    std::string kExtensometerTopic;
    std::string kIMUaxTopic;
    std::string kIMUayTopic;
    std::string kIMUrTopic;
    std::string kInversorTopic;
    std::string kAsStatusTopic;
    std::string kFLWheelSpeedTopic;
    std::string kFRWheelSpeedTopic;
    std::string kRLWheelSpeedTopic;
    std::string kRRWheelSpeedTopic;
    std::string kAmiTopic;
    std::string kTargetSpeedTopic;
    std::string kCmdTopic;
    std::string kLapCountTopic;
    std::string kPerceptionMap;
    std::string kSlamMap;
    std::string kArussimGroundTruthTopic;

    std::string kStateTopic;
    std::string kCarInfoTopic;
    std::string kRunCheckTopic;
    std::string kSteerCheckTopic;
    std::string kBrakingProcedureTopic;


    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_imu_msg_time_;
    rclcpp::Time last_extensometer_msg_time_;
    rclcpp::Time last_fl_ws_msg_time_;
    rclcpp::Time last_fr_ws_msg_time_;
    rclcpp::Time last_rl_ws_msg_time_;
    rclcpp::Time last_rr_ws_msg_time_;
    rclcpp::Time last_inv_speed_msg_time_;
    rclcpp::Time last_cones_count_actual_msg_time_;
    rclcpp::Time last_cones_count_all_msg_time_;


    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr extensometer_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fl_wheelspeed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fr_wheelspeed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rl_wheelspeed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rr_wheelspeed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr inv_speed_sub_;
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr arussim_ground_truth_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr as_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ami_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_speed_sub_;
    rclcpp::Subscription<common_msgs::msg::Cmd>::SharedPtr target_delta_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lap_count_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cones_count_actual_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cones_count_all_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ax_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ay_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr r_sub_;


    // Publishers
    rclcpp::Publisher<common_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<common_msgs::msg::CarInfo>::SharedPtr car_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr run_check_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr steer_check_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr braking_procedure_pub_;


    /**
     * @brief Periodically processes sensor data, estimates vehicle speed, 
     *        and publishes the current state and car information.
     */
    void on_timer();



    /**
     * @brief Callback that receives the current status of the Autonomous System (AS).
     */
    void as_status_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives and validates longitudinal acceleration from the IMU.
     */
    void ax_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives and validates lateral acceleration from the IMU.
     */
    void ay_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives and validates yaw rate (rotation) from the IMU.
     */
    void r_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives steering angle from the extensometer and computes its derivative.
     */
    void extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives and checks front-left wheel speed.
     */
    void fl_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives and checks front-right wheel speed.
     */
    void fr_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives and checks rear-left wheel speed.
     */
    void rl_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives and checks rear-right wheel speed.
     */
    void rr_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives and checks vehicle speed from the inverter.
     */
    void inv_speed_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives ground-truth state from the ARUSSim simulator.
     */
    void arussim_ground_truth_callback(const common_msgs::msg::State::SharedPtr msg);

    /**
     * @brief Callback that receives AMI status from the system interface.
     */
    void ami_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Callback that receives the target speed from the controller.
     */
    void target_speed_callback(const std_msgs::msg::Float32 msg);

    /**
     * @brief Callback that receives the steering command (target delta) from the controller.
     */
    void target_delta_callback(const common_msgs::msg::Cmd msg);

    /**
     * @brief Callback that receives the current lap count from SLAM and may trigger braking.
     */
    void lap_count_callback(const std_msgs::msg::Int16 msg);

    /**
     * @brief Callback that receives actual perceived cones and checks plausibility.
     */
    void cones_count_actual_callback(const sensor_msgs::msg::PointCloud2 msg);

    /**
     * @brief Callback that receives all known cones from SLAM and checks plausibility.
     */
    void cones_count_all_callback(const sensor_msgs::msg::PointCloud2 msg);


    
    /**
     * @brief Retrieves the vehicle's current position and orientation from TF.
     * Uses the transform between "arussim/world" and "slam/vehicle".
     */
    void get_tf_position();
};