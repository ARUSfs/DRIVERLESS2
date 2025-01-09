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

    this->declare_parameter<std::string>("mission", "autocross");
    this->get_parameter("mission", kMission);


    state_pub_ = this->create_publisher<common_msgs::msg::State>(
        "/car_state/state", 1);
    car_info_pub_ = this->create_publisher<common_msgs::msg::CarInfo>(
        "/car_state/car_info", 1);
    as_check_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/car_state/AS_check", 1);

    if(kSimulation && get_arussim_ground_truth){
    arussim_ground_truth_sub_ = this->create_subscription<common_msgs::msg::State>(
        "/arussim_interface/arussim_ground_truth", 1, std::bind(&CarState::
            arussim_ground_truth_callback, this, std::placeholders::_1));
    }

    if(kSimulation){
        extensometer_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/arussim/extensometer", 1, std::bind(&CarState::
                extensometer_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/arussim/imu", 1, std::bind(&CarState::
                imu_callback, this, std::placeholders::_1));

        wheel_speeds_sub_ = this->create_subscription<common_msgs::msg::FourWheelDrive>(
            "/arussim_interface/wheel_speeds", 1, std::bind(&CarState::
                wheel_speeds_callback, this, std::placeholders::_1));
    } else {
        extensometer_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can/extensometer", 1, std::bind(&CarState::
                extensometer_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/can/IMU", 1, std::bind(&CarState::
                imu_callback, this, std::placeholders::_1));

        inv_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can/inv_speed", 1, std::bind(&CarState::
                inv_speed_callback, this, std::placeholders::_1));
        
        as_status_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/can/AS_status", 1, std::bind(&CarState::
                as_status_callback, this, std::placeholders::_1));
    }

    // Configure timer once in the constructor based on the selected controller and frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 100)),
        std::bind(&CarState::on_timer, this));


    // Create TF broadcaster
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize kalman filters
    CarState::initialize_vx_filter();
}

void CarState::as_status_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    as_status_ = msg->data;
}

void CarState::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{   
    if(kSimulation){
        ax_ = msg-> linear_acceleration.x;
        ay_ = msg-> linear_acceleration.y;
        r_ = msg->angular_velocity.z;
    } else {
        ax_ = - msg-> linear_acceleration.x;
        ay_ = msg-> linear_acceleration.y;
        r_ = - msg->angular_velocity.z;
    }
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
    if (kMission!="inspection"){
        this->get_tf_position();
    }
       
    // Estimate vx
    if(kSimulation){
        VectorXd u(1), z(1);
        u << ax_;
        z << (v_front_right_ + v_front_left_ + v_rear_right_ + v_rear_left_)/4;
        vx_filter_.estimate_state(u, z);
        vx_ = vx_filter_.get_estimated_state()(0);
    }

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

    state_pub_->publish(state_msg);


    // Publish car info message
    auto car_info_msg = common_msgs::msg::CarInfo();
    
    car_info_msg.as_status = as_status_;
    car_info_msg.ami = ami_;
    car_info_msg.ebs_status = ebs_status_;
    car_info_msg.ebs_redundancy_status = ebs_redundancy_status_;
    car_info_msg.x = x_;      
    car_info_msg.y = y_;    
    car_info_msg.yaw = yaw_;  
    car_info_msg.vx = vx_;    
    car_info_msg.vy = vy_;
    car_info_msg.target_speed = target_speed_;
    car_info_msg.r = r_;      
    car_info_msg.ax = ax_;
    car_info_msg.ay = ay_;
    car_info_msg.delta = delta_;
    car_info_msg.target_delta = target_delta_;
    car_info_msg.steering_state = steering_state_;
    car_info_msg.torque_actual = torque_actual_;
    car_info_msg.torque_target = torque_target_;
    car_info_msg.brake_hydr_pressure = brake_hydr_pressure_;
    car_info_msg.lap_count = lap_count_;
    car_info_msg.cones_count_actual = cones_count_actual_;
    car_info_msg.cones_count_all = cones_count_all_;

    car_info_pub_->publish(car_info_msg);


    // Publish AS check
    auto as_check_msg = std_msgs::msg::Bool();
    if(kSimulation){
        as_check_msg.data = true;
    } else {
        as_check_msg.data = as_status_ == 2;
    }
    as_check_pub_->publish(as_check_msg);

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

void CarState::initialize_vx_filter(){
    // Set problem size
    int n = 1;
    int m = 1;
    int p = 1;
    vx_filter_.set_problem_size(n, m, p);
    
    // Set initial state and covariance
    VectorXd x_initial(n);
    x_initial << vx_;
    MatrixXd P_initial(n, n); 
    P_initial << 0.01;
    VectorXd u_initial(m);
    u_initial << ax_;
    vx_filter_.set_initial_data(x_initial, P_initial, u_initial);

    // Set process matrices
    MatrixXd M(n, n), B(n, m), Q(n, n);
    M << 0;
    B << 1;
    Q << 0.1;
    vx_filter_.set_process_matrices(M, B, Q);

    // Set measurement matrices
    MatrixXd H(p, n), R(p, p);
    H << 1;
    R << 0.5;
    vx_filter_.set_measurement_matrices(H, M);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarState>());
    rclcpp::shutdown();
    return 0;
}

