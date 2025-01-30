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
    this->declare_parameter<bool>("simulation", false);
    this->declare_parameter<std::string>("mission", "autocross");

    this->declare_parameter<double>("dt_threshold_imu", 0.05);
    this->declare_parameter<double>("dt_threshold_extensometer", 0.05);
    this->declare_parameter<double>("dt_threshold_fl", 0.05);
    this->declare_parameter<double>("dt_threshold_fr", 0.05);
    this->declare_parameter<double>("dt_threshold_rl", 0.05);
    this->declare_parameter<double>("dt_threshold_rr", 0.05);
    this->declare_parameter<double>("dt_threshold_inv", 0.05);
    this->declare_parameter<double>("dt_threshold_cones_count_actual", 0.25);
    this->declare_parameter<double>("dt_threshold_cones_count_all", 0.25);

    this->declare_parameter<double>("max_ax", 15.0);
    this->declare_parameter<double>("max_ay", 5.0);
    this->declare_parameter<double>("max_r", 3.0);
    this->declare_parameter<double>("max_v_front_left", 75.0);
    this->declare_parameter<double>("max_v_front_right", 75.0);
    this->declare_parameter<double>("max_v_rear_left", 75.0);
    this->declare_parameter<double>("max_v_rear_right", 75.0);
    this->declare_parameter<double>("max_vx", 75.0);
    this->declare_parameter<double>("max_plausability_error", 5.0);
    
    // Retrieve parameters after declaration
    this->get_parameter("get_arussim_ground_truth", get_arussim_ground_truth);
    this->get_parameter("simulation", kSimulation);
    this->get_parameter("mission", kMission);

    this->get_parameter("dt_threshold_imu", kThresholdImu);
    this->get_parameter("dt_threshold_extensometer", kThresholdExtensometer);
    this->get_parameter("dt_threshold_fl", kThresholdFl);
    this->get_parameter("dt_threshold_fr", kThresholdFr);
    this->get_parameter("dt_threshold_rl", kThresholdRl);
    this->get_parameter("dt_threshold_rr", kThresholdRr);
    this->get_parameter("dt_threshold_inv", kThresholdInv);
    this->get_parameter("dt_threshold_cones_count_actual", kThresholdConesCountActual);
    this->get_parameter("dt_threshold_cones_count_all", kThresholdConesCountAll);

    this->get_parameter("max_ax", kMaxAx);
    this->get_parameter("max_ay", kMaxAy);
    this->get_parameter("max_r", kMaxR);
    this->get_parameter("max_v_front_left", kMaxVFrontLeft);
    this->get_parameter("max_v_front_right", kMaxVFrontRight);
    this->get_parameter("max_v_rear_left", kMaxVRearLeft);
    this->get_parameter("max_v_rear_right", kMaxVRearRight);
    this->get_parameter("max_vx", kMaxVx);
    this->get_parameter("max_plausability_error", kMaxPlausabilityError);


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

        perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/arussim/perception", 1, std::bind(&CarState::
                perception_callback, this, std::placeholders::_1));
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

    plausability_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 10)),
        std::bind(&CarState::on_plausability_timer, this));

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

void CarState::perception_callback(const sensor_msgs::msg::PointCloud2 msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_cones_count_actual_msg_time_).seconds();
    last_cones_count_actual_msg_time_ = now_time;

    if(dt > kThresholdConesCountActual) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "Perception dt: %f", dt);
    }
    
    cones_count_actual_ = msg.width;

    if (cones_count_actual_ == 0) {
        plausability_ += 2;
        RCLCPP_ERROR(this->get_logger(), "No cones detected by perception");
    }
}

// TODO publish imu separated in arussim
void CarState::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{   
    auto now_time = this->now();
    double dt = (now_time - last_imu_msg_time_).seconds();
    last_imu_msg_time_ = now_time;

    if(dt > kThresholdImu) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "IMU dt: %f", dt);
    }
    
    ax_ = msg-> linear_acceleration.x;
    ay_ = msg-> linear_acceleration.y;
    r_ = msg->angular_velocity.z;

    if (ax_ < -kMaxAx || ax_ > kMaxAx) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "ax_ out of range: %f", ax_);
    }
    if (ay_ < -kMaxAy || ay_ > kMaxAy) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "ay_ out of range: %f", ay_);
    }
    if (r_ < -kMaxR || r_ > kMaxR) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "r_ out of range: %f", r_);
    }
}

void CarState::ax_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_imu_msg_time_).seconds();
    last_imu_msg_time_ = now_time;

    if(dt > kThresholdImu) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "IMU dt: %f", dt);
    }
    
    ax_ = - msg->data; // TODO calibrate IMU

    if (ax_ < -kMaxAx || ax_ > kMaxAx) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "ax_ out of range: %f", ax_);
    }
}

void CarState::ay_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    ay_ = msg->data;

    if (ay_ < -kMaxAy || ay_ > kMaxAy) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "Mechanically impossible ay: %f", ay_);
    }
}

void CarState::r_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    r_ = - msg->data; // TODO calibrate IMU

    if (r_ < -kMaxR || r_ > kMaxR) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "Mechanically impossible r: %f", r_);
    }
}

void CarState::extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_extensometer_msg_time_).seconds();
    last_extensometer_msg_time_ = now_time;
    epos_OK_= true;

    if(dt > kThresholdExtensometer) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "Extensometer dt: %f", dt);
    }
    
    delta_ = msg->data;

    if (delta_ < -21 || delta_ > 21) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "delta_ out of range: %f", delta_);
    }
}

void CarState::fl_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_fl_ws_msg_time_).seconds();
    last_fl_ws_msg_time_ = now_time;

    if(dt > kThresholdFl) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "FL wheelspeed dt: %f", dt);
    }

    v_front_left_ = 1/msg->data;

    if (v_front_left_ > kMaxVFrontLeft) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "v_front_left_ exceeds max: %f", v_front_left_);
    }
}

void CarState::fr_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_fr_ws_msg_time_).seconds();
    last_fr_ws_msg_time_ = now_time;

    if(dt > kThresholdFr) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "FR wheelspeed dt: %f", dt);
    }

    v_front_right_ = 1/msg->data;

    if (v_front_right_ > kMaxVFrontRight) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "v_front_right_ exceeds max: %f", v_front_right_);
    }
}

void CarState::rl_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_rl_ws_msg_time_).seconds();
    last_rl_ws_msg_time_ = now_time;

    if(dt > kThresholdRl) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "RL wheelspeed dt: %f", dt);
    }

    v_rear_left_ = 1/msg->data;

    if (v_rear_left_ > kMaxVRearLeft) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "v_rear_left_ exceeds max: %f", v_rear_left_);
    }
}

void CarState::rr_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_rr_ws_msg_time_).seconds();
    last_rr_ws_msg_time_ = now_time;

    if(dt > kThresholdRr) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "RR wheelspeed dt: %f", dt);
    }

    v_rear_right_ = 1/msg->data;

    if (v_rear_right_ > kMaxVRearRight) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "v_rear_right_ exceeds max: %f", v_rear_right_);
    }
}

// TODO publish wheel speeds separated in arussim
void CarState::wheel_speeds_callback(const common_msgs::msg::FourWheelDrive::SharedPtr msg)
{
    v_front_right_ = msg-> front_right;
    v_front_left_ = msg-> front_left;
    v_rear_right_ = msg-> rear_right;
    v_rear_left_ = msg-> rear_left;
    inv_OK_ = true;
}

void CarState::inv_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_inv_speed_msg_time_).seconds();
    last_inv_speed_msg_time_ = now_time;
    inv_OK_ = true;

    if(dt > kThresholdInv) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "Inv speed dt: %f", dt);
    }

    vx_ = msg->data;

    if (vx_ > kMaxVx || vx_ < -0.5) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "vx_ exceeds max: %f", vx_);
    }
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

    if(dt > kThresholdConesCountActual) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "Cones count actual dt: %f", dt);
    }

    cones_count_actual_ = msg.width;

    if (cones_count_actual_ <= 0) {
        plausability_+= 2;
        RCLCPP_ERROR(this->get_logger(), "No cones detected by perception");
    }
}

void CarState::cones_count_all_callback(const sensor_msgs::msg::PointCloud2 msg)
{   
    auto now_time = this->now();
    double dt = (now_time - last_cones_count_all_msg_time_).seconds();
    last_cones_count_all_msg_time_ = now_time;

    if(dt > kThresholdConesCountAll) {
        plausability_ += 2 ;
        RCLCPP_ERROR(this->get_logger(), "Cones count all dt: %f", dt);
    }

    cones_count_all_ = msg.width;

    if (cones_count_all_ <= 0) {
        plausability_+= 2;
        RCLCPP_ERROR(this->get_logger(), "No cones detected by slam");
    }
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
    if (plausability_OK_ && epos_OK_ && inv_OK_){
        if(kSimulation){
            run_check_msg.data = true;
        } else {
            run_check_msg.data = as_status_ == 3;
        }
        run_check_pub_->publish(run_check_msg);
    }

    // Publish steering check
    auto steering_check_msg = std_msgs::msg::Bool();
    steering_check_msg.data = run_check_msg.data && (vx_ >= 0.5);
    steer_check_pub_->publish(steering_check_msg);
}

void CarState::on_plausability_timer()
{
    // Debug
    std::cout << "Plausability: " << plausability_ << std::endl;

    prev_plausability_ = plausability_;

    if (plausability_ > kMaxPlausabilityError){
        if (as_status_ == 3){
            as_status_ = 4;
            RCLCPP_ERROR(this->get_logger(), "Plausability error, AS Emergency");
        } else {
            plausability_OK_ = false;
            RCLCPP_ERROR(this->get_logger(), "Plausability error, plausability_OK_ = false");
        }
    }
    if (plausability_ > 0){
        plausability_--;
    }
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

