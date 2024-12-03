/** 
 * @file car_state.cpp
 * @brief CarState node implementaion for ARUS Team Driverless pipeline
*/

#include "car_state/car_state.hpp"

/**
 * @class CarState
 * @brief CarState class 
 * 
 * Node that collects car data, estimates parameters, and publishes them on /car_state/State.
 */

CarState::CarState(): Node("car_state")
{   
    bool get_arussim_ground_truth;
    this->declare_parameter<bool>("get_arussim_ground_truth", false);
    this->get_parameter("get_arussim_ground_truth", get_arussim_ground_truth);

    this->declare_parameter<bool>("simulation", false);
    this->get_parameter("simulation", kSimulation);


    pub_state_ = this->create_publisher<common_msgs::msg::State>(
        "/car_state/state", 1);

    if(kSimulation && get_arussim_ground_truth){
    sub_arussim_ground_truth_ = this->create_subscription<common_msgs::msg::State>(
        "/arussim_interface/arussim_ground_truth", 1, std::bind(&CarState::
            arussim_ground_truth_callback, this, std::placeholders::_1));
    }

    if(kSimulation){
        sub_extensometer_ = this->create_subscription<std_msgs::msg::Float32>(
            "/arussim/extensometer", 1, std::bind(&CarState::
                extensometer_callback, this, std::placeholders::_1));

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/arussim/imu", 1, std::bind(&CarState::
                imu_callback, this, std::placeholders::_1));

        sub_wheel_speeds_ = this->create_subscription<common_msgs::msg::FourWheelDrive>(
            "/arussim_interface/wheel_speeds", 1, std::bind(&CarState::
                wheel_speeds_callback, this, std::placeholders::_1));
    } else {
        sub_extensometer_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can/extensometer", 1, std::bind(&CarState::
                extensometer_callback, this, std::placeholders::_1));

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/can/IMU", 1, std::bind(&CarState::
                imu_callback, this, std::placeholders::_1));

        sub_inv_speed_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can/inv_speed", 1, std::bind(&CarState::
                inv_speed_callback, this, std::placeholders::_1));
    }

    // Configure timer once in the constructor based on the selected controller and frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 100)),
        std::bind(&CarState::on_timer, this));


    //Create estimation object
    state_estimation_ = Estimation();

    // Create TF broadcaster
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

}

void CarState::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    ax_ = msg-> linear_acceleration.x;
    ay_ = msg-> linear_acceleration.y;
    r_ = msg->angular_velocity.z;
}

void CarState::extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    delta_ = msg->data;
}

void CarState::wheel_speeds_callback(const common_msgs::msg::FourWheelDrive::SharedPtr msg)
{
    v_front_right_ = msg-> front_right;
    v_front_left_ = msg-> front_left;
    v_rear_right_ = msg-> rear_right;
    v_rear_left_ = msg-> rear_left;

    vx_ = (v_front_right_ + v_front_left_ + v_rear_right_ + v_rear_left_)/4;
}

void CarState::inv_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    vx_ = msg->data;
}

void CarState::arussim_ground_truth_callback(const common_msgs::msg::State::SharedPtr msg)
{
    x_ = msg->x;
    y_ = msg->y;
    yaw_ = msg->yaw;
}

void CarState::on_timer()
{
    this->get_tf_position();

    // Estimate velocity
    state_estimation_.set_measurement_data(v_front_right_, v_front_left_, v_rear_right_, v_rear_left_, ax_, ay_);

    Vector2d v_est = state_estimation_.kalman_velocity_estimation();
    // vx_ = v_est(0);
    // vy_ = v_est(1);

    // Publish state message
    auto state_msg = common_msgs::msg::State();
    
    state_msg.x = x_;      
    state_msg.y = y_;    
    state_msg.yaw = yaw_;  
    state_msg.vx = vx_;    
    state_msg.vy = vy_;
    state_msg.r = r_;      
    state_msg.ax = ax_;
    state_msg.ay = ay_;
    state_msg.delta = delta_;

    pub_state_->publish(state_msg);

}

void CarState::get_tf_position()
{
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform("arussim/world", "slam/vehicle", tf2::TimePointZero);
        tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        x_ = transform.transform.translation.x;
        y_ = transform.transform.translation.y;
        yaw_ = yaw;
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform not available: %s", ex.what());
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarState>());
    rclcpp::shutdown();
    return 0;
}

