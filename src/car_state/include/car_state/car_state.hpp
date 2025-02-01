/**
 * @file car_state.hpp
 * @brief CarState node header for ARUS Team Driverless pipeline
 */

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/car_info.hpp"
#include "common_msgs/msg/four_wheel_drive.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "common_msgs/msg/cmd.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "car_state/kalman_filter.hpp"

/**
 * @class CarState
 * @brief CarState class 
 * 
 * Header node car_state
 * 
 */

class CarState : public rclcpp::Node
{
public:
    CarState();

private:
    // Callback for the subscription
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void fl_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void fr_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void rl_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void rr_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void wheel_speeds_callback(const common_msgs::msg::FourWheelDrive::SharedPtr msg);
    void inv_speed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void arussim_ground_truth_callback(const common_msgs::msg::State::SharedPtr msg);
    void as_status_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void ax_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void ay_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void r_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void target_speed_callback(const std_msgs::msg::Float32 msg);
    void target_delta_callback(const common_msgs::msg::Cmd msg);
    void lap_count_callback(const std_msgs::msg::Int16 msg);
    void cones_count_actual_callback(const sensor_msgs::msg::PointCloud2 msg);
    void cones_count_all_callback(const sensor_msgs::msg::PointCloud2 msg);
    void ami_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void perception_callback(const sensor_msgs::msg::PointCloud2 msg);
    

    // Functions
    void on_timer();
    void get_tf_position();
    void initialize_vx_filter();

    // Estimation filters
    KalmanFilter vx_filter_;

    // Private attributes to store received data
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

    int as_status_ = 0;
    int ami_ = 0;
    int ebs_status_ = 0;
    int ebs_redundancy_status_ = 0;
    double target_speed_ = 0;
    double target_delta_ = 0;
    int steering_state_ = 0;
    double torque_actual_ = 0;
    double torque_target_ = 0;
    double brake_hydr_pressure_ = 0;
    int lap_count_ = 0;
    int cones_count_actual_ = 0;
    int cones_count_all_ = 0;
    double plausability_ = 0;
    double prev_plausability_ = 0;
    bool plausability_OK_ = true;
    bool epos_OK_ = false;
    bool inv_OK_ = false;

    double kThresholdImu;
    double kThresholdExtensometer;
    double kThresholdFl;
    double kThresholdFr;
    double kThresholdRl;
    double kThresholdRr;
    double kThresholdInv;
    double kThresholdConesCountActual;
    double kThresholdConesCountAll;
    double kMaxError;

    bool kSimulation;
    std::string kMission;

    // Maximum threshold parameters
    double kMaxAx;
    double kMaxAy;
    double kMaxR;
    double kMaxVFrontLeft;
    double kMaxVFrontRight;
    double kMaxVRearLeft;
    double kMaxVRearRight;
    double kMaxVx;
    double kMaxPlausabilityError;

    //Error weights
    double kErrorWeightAx;
    double kErrorWeightAy;
    double kErrorWeightR;
    double kErrorWeightExtensometer;
    double kErrorWeightWheelSpeed;
    double kErrorWeightInvSpeed;
    double kErrorWeightConesCountActual;
    double kErrorWeightConesCountAll;    

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr extensometer_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fl_wheelspeed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fr_wheelspeed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rl_wheelspeed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rr_wheelspeed_sub_;
    rclcpp::Subscription<common_msgs::msg::FourWheelDrive>::SharedPtr wheel_speeds_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr inv_speed_sub_;
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr arussim_ground_truth_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr as_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ami_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_speed_sub_;
    rclcpp::Subscription<common_msgs::msg::Cmd>::SharedPtr target_delta_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lap_count_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cones_count_actual_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cones_count_all_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ax_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ay_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr r_sub_;

    // Publisher for the aggregated state
    rclcpp::Publisher<common_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<common_msgs::msg::CarInfo>::SharedPtr car_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr run_check_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr steer_check_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Time-tracking members
    rclcpp::Time last_imu_msg_time_;
    rclcpp::Time last_extensometer_msg_time_;
    rclcpp::Time last_fl_ws_msg_time_;
    rclcpp::Time last_fr_ws_msg_time_;
    rclcpp::Time last_rl_ws_msg_time_;
    rclcpp::Time last_rr_ws_msg_time_;
    rclcpp::Time last_inv_speed_msg_time_;
    rclcpp::Time last_cones_count_actual_msg_time_;
    rclcpp::Time last_cones_count_all_msg_time_;
};