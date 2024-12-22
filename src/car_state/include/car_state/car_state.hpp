/**
 * @file car_state.hpp
 * @brief CarState node header for ARUS Team Driverless pipeline
 */

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "common_msgs/msg/state.hpp"
#include "common_msgs/msg/four_wheel_drive.hpp"

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
    void wheel_speeds_callback(const common_msgs::msg::FourWheelDrive::SharedPtr msg);
    void inv_speed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void arussim_ground_truth_callback(const common_msgs::msg::State::SharedPtr msg);
    void as_status_callback(const std_msgs::msg::Int16::SharedPtr msg);

    // Functions
    void on_timer();
    void get_tf_position();
    KalmanFilter initialize_vx_filter();

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

    bool kSimulation;
    std::string kMission;

    // AS status
    // 0: AS off
    // 1: AS ready
    // 2: AS driving
    // 3: AS finished
    // 4: AS emergency
    int as_status_ = 0;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr extensometer_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<common_msgs::msg::FourWheelDrive>::SharedPtr wheel_speeds_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr inv_speed_sub_;
    rclcpp::Subscription<common_msgs::msg::State>::SharedPtr arussim_ground_truth_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr as_status_sub_;

    // Publisher for the aggregated state
    rclcpp::Publisher<common_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr as_check_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};