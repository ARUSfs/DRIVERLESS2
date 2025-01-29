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

    this->declare_parameter<double>("dt_threshold_imu", 0.05);
    this->get_parameter("dt_threshold_imu", dt_threshold_imu_);

    this->declare_parameter<double>("dt_threshold_extensometer", 0.05);
    this->get_parameter("dt_threshold_extensometer", dt_threshold_extensometer_);

    this->declare_parameter<double>("dt_threshold_fl", 0.05);
    this->get_parameter("dt_threshold_fl", dt_threshold_fl_);

    this->declare_parameter<double>("dt_threshold_fr", 0.05);
    this->get_parameter("dt_threshold_fr", dt_threshold_fr_);

    this->declare_parameter<double>("dt_threshold_rl", 0.05);
    this->get_parameter("dt_threshold_rl", dt_threshold_rl_);

    this->declare_parameter<double>("dt_threshold_rr", 0.05);
    this->get_parameter("dt_threshold_rr", dt_threshold_rr_);

    this->declare_parameter<double>("dt_threshold_inv", 0.05);
    this->get_parameter("dt_threshold_inv", dt_threshold_inv_);

    this->declare_parameter<double>("dt_threshold_cones_count_actual", 0.05);
    this->get_parameter("dt_threshold_cones_count_actual", dt_threshold_cones_count_actual_);

    this->declare_parameter<double>("dt_threshold_cones_count_all", 0.05);
    this->get_parameter("dt_threshold_cones_count_all", dt_threshold_cones_count_all_);


    state_pub_ = this->create_publisher<common_msgs::msg::State>(
        "/car_state/state", 1);
    car_info_pub_ = this->create_publisher<common_msgs::msg::CarInfo>(
        "/car_state/car_info", 1);
    run_check_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/car_state/run_check", 1);
    steer_check_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/car_state/steer_check", 1);


    ami_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/can_interface/AMI", 1, std::bind(&CarState::
            ami_callback, this, std::placeholders::_1));

    target_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/controller/target_speed", 1, std::bind(&CarState::
            target_speed_callback, this, std::placeholders::_1));

    target_delta_sub_ = this->create_subscription<common_msgs::msg::Cmd>(
        "/controller/cmd", 1, std::bind(&CarState::
            target_delta_callback, this, std::placeholders::_1));

    lap_count_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/slam/lap_count", 1, std::bind(&CarState::
            lap_count_callback, this, std::placeholders::_1));

    cones_count_actual_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/map", 1, std::bind(&CarState::
            cones_count_actual_callback, this, std::placeholders::_1));

    cones_count_all_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/slam/map", 1, std::bind(&CarState::
            cones_count_all_callback, this, std::placeholders::_1));


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
            "/can_interface/extensometer", 1, std::bind(&CarState::
                extensometer_callback, this, std::placeholders::_1));

        ax_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can_interface/IMU/ax", 1, std::bind(&CarState::
                ax_callback, this, std::placeholders::_1));
        
        ay_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can_interface/IMU/ay", 1, std::bind(&CarState::
                ay_callback, this, std::placeholders::_1));

        r_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can_interface/IMU/yaw_rate", 1, std::bind(&CarState::
                r_callback, this, std::placeholders::_1));

        inv_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can_interface/inv_speed", 1, std::bind(&CarState::
                inv_speed_callback, this, std::placeholders::_1));
        
        as_status_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can_interface/AS_status", 1, std::bind(&CarState::
                as_status_callback, this, std::placeholders::_1));

        fl_wheelspeed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can_interface/fl_wheel_speed", 1, std::bind(&CarState::
                fl_wheelspeed_callback, this, std::placeholders::_1));
        fr_wheelspeed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can_interface/fr_wheel_speed", 1, std::bind(&CarState::
                fr_wheelspeed_callback, this, std::placeholders::_1));
        rl_wheelspeed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can_interface/rl_wheel_speed", 1, std::bind(&CarState::
                rl_wheelspeed_callback, this, std::placeholders::_1));
        rr_wheelspeed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can_interface/rr_wheel_speed", 1, std::bind(&CarState::
                rr_wheelspeed_callback, this, std::placeholders::_1));
        
    }

    // Configure timer once in the constructor based on the selected controller and frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 100)),
        std::bind(&CarState::on_timer, this));

    last_imu_msg_time_ = this->now();
    last_extensometer_msg_time_ = this->now();
    last_fl_ws_msg_time_ = this->now();
    last_fr_ws_msg_time_ = this->now();
    last_rl_ws_msg_time_ = this->now();
    last_rr_ws_msg_time_ = this->now();
    last_inv_speed_msg_time_ = this->now();
    last_cones_count_actual_msg_time_ = this->now();
    last_cones_count_all_msg_time_ = this->now();

    // Create TF broadcaster
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize kalman filters
    CarState::initialize_vx_filter();
}

void CarState::as_status_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    as_status_ = msg->data +1;

    if(as_status_ == 3)
    {
        brake_hydr_pressure_ = 0;
    }else{
        brake_hydr_pressure_ = 100;
    }
}

// TODO publish imu separated in arussim
void CarState::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{   
    auto now_time = this->now();
    double dt = (now_time - last_imu_msg_time_).seconds();
    last_imu_msg_time_ = now_time;

    if(dt > dt_threshold_imu_) {
        plausibility_++;
        std::cout << "IMU dt: " << dt << std::endl;
    }

    ax_ = msg-> linear_acceleration.x;
    ay_ = msg-> linear_acceleration.y;
    r_ = msg->angular_velocity.z;
}

void CarState::ax_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_imu_msg_time_).seconds();
    last_imu_msg_time_ = now_time;

    if(dt > dt_threshold_imu_) {
        plausibility_++;
        std::cout << "IMU dt: " << dt << std::endl;
    }

    ax_ = - msg->data; // TODO calibrate IMU
}

void CarState::ay_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_imu_msg_time_).seconds();
    std::cout << "IMU dt: " << dt << std::endl;
    last_imu_msg_time_ = now_time;

    ay_ = msg->data;
}

void CarState::r_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_imu_msg_time_).seconds();
    std::cout << "IMU dt: " << dt << std::endl;
    last_imu_msg_time_ = now_time;

    r_ = - msg->data; // TODO calibrate IMU
}

void CarState::extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_extensometer_msg_time_).seconds();
    last_extensometer_msg_time_ = now_time;

    if(dt > dt_threshold_extensometer_) {
        plausibility_++;
        std::cout << "Extensometer dt: " << dt << std::endl;
    }

    delta_ = msg->data;
}

void CarState::fl_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_fl_ws_msg_time_).seconds();
    last_fl_ws_msg_time_ = now_time;

    if(dt > dt_threshold_fl_) {
        plausibility_++;
        std::cout << "FL wheelspeed dt: " << dt << std::endl;
    }

    v_front_left_ = 1/msg->data;
}

void CarState::fr_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_fr_ws_msg_time_).seconds();
    last_fr_ws_msg_time_ = now_time;

    if(dt > dt_threshold_fr_) {
        plausibility_++;
        std::cout << "FR wheelspeed dt: " << dt << std::endl;
    }

    v_front_right_ = 1/msg->data;
    std::cout << "v_front_right_: " << v_front_right_ << std::endl;
}

void CarState::rl_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_rl_ws_msg_time_).seconds();
    last_rl_ws_msg_time_ = now_time;

    if(dt > dt_threshold_rl_) {
        plausibility_++;
        std::cout << "RL wheelspeed dt: " << dt << std::endl;
    }

    v_rear_left_ = 1/msg->data;
}

void CarState::rr_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_rr_ws_msg_time_).seconds();
    last_rr_ws_msg_time_ = now_time;

    if(dt > dt_threshold_rr_) {
        plausibility_++;
        std::cout << "RR wheelspeed dt: " << dt << std::endl;
    }

    v_rear_right_ = 1/msg->data;
}

// TODO publish wheel speeds separated in arussim
void CarState::wheel_speeds_callback(const common_msgs::msg::FourWheelDrive::SharedPtr msg)
{
    v_front_right_ = msg-> front_right;
    v_front_left_ = msg-> front_left;
    v_rear_right_ = msg-> rear_right;
    v_rear_left_ = msg-> rear_left;
}

void CarState::inv_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_inv_speed_msg_time_).seconds();
    last_inv_speed_msg_time_ = now_time;

    if(dt > dt_threshold_inv_) {
        plausibility_++;
        std::cout << "Inv speed dt: " << dt << std::endl;
    }

    vx_ = msg->data;
}

void CarState::arussim_ground_truth_callback(const common_msgs::msg::State::SharedPtr msg)
{
    x_ = msg->x;
    y_ = msg->y;
    yaw_ = msg->yaw;
}

void CarState::ami_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    ami_ = msg->data;
}

void CarState::target_speed_callback(const std_msgs::msg::Float32 msg)
{
    target_speed_ = msg.data;
}

void CarState::target_delta_callback(const common_msgs::msg::Cmd msg)
{
    target_delta_ = msg.delta;
}

void CarState::lap_count_callback(const std_msgs::msg::Int16 msg)
{
    lap_count_ = msg.data;
}

void CarState::cones_count_actual_callback(const sensor_msgs::msg::PointCloud2 msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_cones_count_actual_msg_time_).seconds();
    last_cones_count_actual_msg_time_ = now_time;

    if(dt > dt_threshold_cones_count_actual_) {
        plausibility_++;
        std::cout << "Cones count actual dt: " << dt << std::endl;
    }

    cones_count_actual_ = msg.width;
}

void CarState::cones_count_all_callback(const sensor_msgs::msg::PointCloud2 msg)
{   
    auto now_time = this->now();
    double dt = (now_time - last_cones_count_all_msg_time_).seconds();
    last_cones_count_all_msg_time_ = now_time;

    if(dt > dt_threshold_cones_count_all_) {
        plausibility_++;
        std::cout << "Cones count all dt: " << dt << std::endl;
    }

    cones_count_all_ = msg.width;
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


    if(as_status_== 3 && target_speed_== 0.0 && vx_ < 0.5){ 
        as_status_ = 5;
    }

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


    // Publish run check
    auto run_check_msg = std_msgs::msg::Bool();
    if(kSimulation){
        run_check_msg.data = true;
    } else {
        run_check_msg.data = as_status_ == 3;
    }
    run_check_pub_->publish(run_check_msg);


    // Publish steering check
    auto steering_check_msg = std_msgs::msg::Bool();
    steering_check_msg.data = run_check_msg.data && (vx_ >= 0.5);
    steer_check_pub_->publish(steering_check_msg);

    std::cout << "Plausibility: " << plausibility_ << std::endl;
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
        // RCLCPP_WARN(this->get_logger(), "Transform not available: %s", ex.what());
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

