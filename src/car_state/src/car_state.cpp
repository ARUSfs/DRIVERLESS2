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


    this->declare_parameter<bool>("safe_mode", true);

    this->declare_parameter<double>("dt_threshold_imu", 0.05);
    this->declare_parameter<double>("dt_threshold_extensometer", 0.05);
    this->declare_parameter<double>("dt_threshold_wheelspeed", 0.05);
    this->declare_parameter<double>("dt_threshold_inv", 0.05);
    this->declare_parameter<double>("dt_threshold_cones_count_actual", 0.25);
    this->declare_parameter<double>("dt_threshold_cones_count_all", 0.25);

    this->declare_parameter<double>("max_ax", 15.0);
    this->declare_parameter<double>("max_ay", 5.0);
    this->declare_parameter<double>("max_r", 3.0);
    this->declare_parameter<double>("max_vx", 75.0);
    this->declare_parameter<double>("max_plausability_error", 5.0);

    this->declare_parameter<double>("error_weight_imu", 1.0);
    this->declare_parameter<double>("error_weight_wheel_speed", 1.0);
    this->declare_parameter<double>("error_weight_inv_speed", 1.0);
    this->declare_parameter<double>("error_weight_cones_count_actual", 1.0);
    this->declare_parameter<double>("error_weight_cones_count_all", 1.0);
    this->declare_parameter<double>("error_weight_extensometer", 1.0);

    this->declare_parameter<std::string>("extensometer_topic", "/can_interface/extensometer");
    this->declare_parameter<std::string>("imu_ax_topic", "/can_interface/IMU/ax");
    this->declare_parameter<std::string>("imu_ay_topic", "/can_interface/IMU/ay");
    this->declare_parameter<std::string>("imu_r_topic", "/can_interface/IMU/r");
    this->declare_parameter<std::string>("inversor_topic", "/can_interface/inv_speed");
    this->declare_parameter<std::string>("as_status_topic", "/can_interface/AS_status");
    this->declare_parameter<std::string>("fl_wheel_speed_topic", "/can_interface/fl_wheel_speed");
    this->declare_parameter<std::string>("fr_wheel_speed_topic", "/can_interface/fr_wheel_speed");
    this->declare_parameter<std::string>("rl_wheel_speed_topic", "/can_interface/rl_wheel_speed");
    this->declare_parameter<std::string>("rr_wheel_speed_topic", "/can_interface/rr_wheel_speed");

    this->get_parameter("get_arussim_ground_truth", get_arussim_ground_truth);
    this->get_parameter("simulation", kSimulation);
    this->get_parameter("mission", kMission);

    this->get_parameter("safe_mode", kSafeMode);

    // Parameters for safe mode plausability checks
    this->get_parameter("dt_threshold_imu", kThresholdImu);
    this->get_parameter("dt_threshold_extensometer", kThresholdExtensometer);
    this->get_parameter("dt_threshold_wheelspeed", kThresholdWheelSpeed);
    this->get_parameter("dt_threshold_inv", kThresholdInv);
    this->get_parameter("dt_threshold_cones_count_actual", kThresholdConesCountActual);
    this->get_parameter("dt_threshold_cones_count_all", kThresholdConesCountAll);

    this->get_parameter("max_ax", kMaxAx);
    this->get_parameter("max_ay", kMaxAy);
    this->get_parameter("max_r", kMaxR);
    this->get_parameter("max_vx", kMaxVx);
    this->get_parameter("max_plausability_error", kMaxPlausabilityError);

    this->get_parameter("error_weight_imu", kErrorWeightIMU);
    this->get_parameter("error_weight_extensometer", kErrorWeightExtensometer);
    this->get_parameter("error_weight_wheel_speed", kErrorWeightWheelSpeed);
    this->get_parameter("error_weight_inv_speed", kErrorWeightInvSpeed);
    this->get_parameter("error_weight_cones_count_actual", kErrorWeightConesCountActual);
    this->get_parameter("error_weight_cones_count_all", kErrorWeightConesCountAll);

    this->get_parameter("extensometer_topic", kExtensometerTopic);
    this->get_parameter("imu_ax_topic", kIMUaxTopic);
    this->get_parameter("imu_ay_topic", kIMUayTopic);
    this->get_parameter("imu_r_topic", kIMUrTopic);
    this->get_parameter("inversor_topic", kInversorTopic);
    this->get_parameter("as_status_topic", kAsStatusTopic);
    this->get_parameter("fl_wheel_speed_topic", kFLWheelSpeedTopic);
    this->get_parameter("fr_wheel_speed_topic", kFRWheelSpeedTopic);
    this->get_parameter("rl_wheel_speed_topic", kRLWheelSpeedTopic);
    this->get_parameter("rr_wheel_speed_topic", kRRWheelSpeedTopic);


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

    extensometer_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        kExtensometerTopic, 1, std::bind(&CarState::
            extensometer_callback, this, std::placeholders::_1));

    ax_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        kIMUaxTopic, 1, std::bind(&CarState::
            ax_callback, this, std::placeholders::_1));
    
    ay_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        kIMUayTopic, 1, std::bind(&CarState::
            ay_callback, this, std::placeholders::_1));

    r_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        kIMUrTopic, 1, std::bind(&CarState::
            r_callback, this, std::placeholders::_1));

    inv_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        kInversorTopic, 1, std::bind(&CarState::
            inv_speed_callback, this, std::placeholders::_1));
    
    as_status_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        kAsStatusTopic, 1, std::bind(&CarState::
            as_status_callback, this, std::placeholders::_1));

    fl_wheelspeed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        kFLWheelSpeedTopic, 1, std::bind(&CarState::
            fl_wheelspeed_callback, this, std::placeholders::_1));
    fr_wheelspeed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        kFRWheelSpeedTopic, 1, std::bind(&CarState::
            fr_wheelspeed_callback, this, std::placeholders::_1));
    rl_wheelspeed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        kRLWheelSpeedTopic, 1, std::bind(&CarState::
            rl_wheelspeed_callback, this, std::placeholders::_1));
    rr_wheelspeed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        kRRWheelSpeedTopic, 1, std::bind(&CarState::
            rr_wheelspeed_callback, this, std::placeholders::_1));

    // Configure timer once in the constructor based on the selected controller and frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 100)),
        std::bind(&CarState::on_timer, this));

    // Initialize last message times
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

    // Initialize speed estimator
    speed_estimator_.initialize_speed_estimator();

    // Start simulation in Driving mode
    if(kSimulation){
        as_status_ = 3;
    }
}

void CarState::as_status_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    as_status_ = msg->data; 

    if(as_status_ == 3)
    {
        brake_hydr_pressure_ = 0;
    }else{
        brake_hydr_pressure_ = 100;
    }
}

void CarState::ax_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_imu_msg_time_).seconds();
    last_imu_msg_time_ = now_time;

    if (kSafeMode && (dt > kThresholdImu)) {
        plausability_ += kErrorWeightIMU;
        RCLCPP_ERROR(this->get_logger(), "IMU dt: %f", dt);
    }
    
    ax_ = msg->data;

    if (kSafeMode && (ax_ < -kMaxAx || ax_ > kMaxAx)) {
        plausability_ += kErrorWeightIMU;
        RCLCPP_ERROR(this->get_logger(), "ax_ out of range: %f", ax_);
    }
}

void CarState::ay_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    ay_ = msg->data;

    if (kSafeMode && (ay_ < -kMaxAy || ay_ > kMaxAy)) {
        plausability_ += kErrorWeightIMU;
        RCLCPP_ERROR(this->get_logger(), "Mechanically impossible ay: %f", ay_);
    }
}

void CarState::r_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    r_ = msg->data;

    if (kSafeMode && (r_ < -kMaxR || r_ > kMaxR)) {
        plausability_ += kErrorWeightIMU;
        RCLCPP_ERROR(this->get_logger(), "Mechanically impossible r: %f", r_);
    }
}

void CarState::extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_extensometer_msg_time_).seconds();
    last_extensometer_msg_time_ = now_time;

    if (kSafeMode && dt > kThresholdExtensometer) {
        plausability_ += kErrorWeightExtensometer;
        RCLCPP_ERROR(this->get_logger(), "Extensometer dt: %f", dt);
    }
    
    delta_der_ = (msg->data - delta_) / dt;
    delta_ = msg->data;

    if (kSafeMode && (delta_ < -21 || delta_ > 21)) {
        plausability_ += kErrorWeightExtensometer;
        RCLCPP_ERROR(this->get_logger(), "delta_ out of range: %f", delta_);
    }
}

void CarState::fl_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_fl_ws_msg_time_).seconds();
    last_fl_ws_msg_time_ = now_time;

    if (kSafeMode && dt > kThresholdWheelSpeed) {
        plausability_ += kErrorWeightWheelSpeed;
        RCLCPP_ERROR(this->get_logger(), "FL wheelspeed dt: %f", dt);
    }

    v_front_left_ = msg->data;

    if (kSafeMode && v_front_left_ > kMaxVx) {
        plausability_ += kErrorWeightWheelSpeed;
        RCLCPP_ERROR(this->get_logger(), "v_front_left_ exceeds max: %f", v_front_left_);
    }
}

void CarState::fr_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_fr_ws_msg_time_).seconds();
    last_fr_ws_msg_time_ = now_time;

    if (kSafeMode && dt > kThresholdWheelSpeed) {
        plausability_ += kErrorWeightWheelSpeed;
        RCLCPP_ERROR(this->get_logger(), "FR wheelspeed dt: %f", dt);
    }

    v_front_right_ = msg->data;

    if (kSafeMode && v_front_right_ > kMaxVx) {
        plausability_ += kErrorWeightWheelSpeed;
        RCLCPP_ERROR(this->get_logger(), "v_front_right_ exceeds max: %f", v_front_right_);
    }
}

void CarState::rl_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_rl_ws_msg_time_).seconds();
    last_rl_ws_msg_time_ = now_time;

    if (kSafeMode && dt > kThresholdWheelSpeed) {
        plausability_ += kErrorWeightWheelSpeed;
        RCLCPP_ERROR(this->get_logger(), "RL wheelspeed dt: %f", dt);
    }

    v_rear_left_ = msg->data;

    if (kSafeMode && v_rear_left_ > kMaxVx) {
        plausability_ += kErrorWeightWheelSpeed;
        RCLCPP_ERROR(this->get_logger(), "v_rear_left_ exceeds max: %f", v_rear_left_);
    }
}

void CarState::rr_wheelspeed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_rr_ws_msg_time_).seconds();
    last_rr_ws_msg_time_ = now_time;

    if (kSafeMode && dt > kThresholdWheelSpeed) {
        plausability_ += kErrorWeightWheelSpeed;
        RCLCPP_ERROR(this->get_logger(), "RR wheelspeed dt: %f", dt);
    }

    v_rear_right_ = msg->data;

    if (kSafeMode && v_rear_right_ > kMaxVx) {
        plausability_ += kErrorWeightWheelSpeed;
        RCLCPP_ERROR(this->get_logger(), "v_rear_right_ exceeds max: %f", v_rear_right_);
    }
}

void CarState::inv_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    auto now_time = this->now();
    double dt = (now_time - last_inv_speed_msg_time_).seconds();
    last_inv_speed_msg_time_ = now_time;

    if(kSafeMode && dt > kThresholdInv) {
        plausability_ += kErrorWeightInvSpeed;
        RCLCPP_ERROR(this->get_logger(), "Inv speed dt: %f", dt);
    }

    inv_speed_ = msg->data;

    if (kSafeMode && (vx_ > kMaxVx || vx_ < -0.5)) {
        plausability_ += kErrorWeightInvSpeed;
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

    if(kSafeMode && dt > kThresholdConesCountActual) {
        plausability_ += kErrorWeightConesCountActual;
        RCLCPP_ERROR(this->get_logger(), "Cones count actual dt: %f", dt);
    }

    cones_count_actual_ = msg.width;

    if (kSafeMode && cones_count_actual_ <= 0) {
        plausability_+= kErrorWeightConesCountActual;
        RCLCPP_ERROR(this->get_logger(), "No cones detected by perception");
    }
}

void CarState::cones_count_all_callback(const sensor_msgs::msg::PointCloud2 msg)
{   
    auto now_time = this->now();
    double dt = (now_time - last_cones_count_all_msg_time_).seconds();
    last_cones_count_all_msg_time_ = now_time;

    if(kSafeMode && dt > kThresholdConesCountAll) {
        plausability_ += kErrorWeightConesCountAll;
        RCLCPP_ERROR(this->get_logger(), "Cones count all dt: %f", dt);
    }

    cones_count_all_ = msg.width;

    if (kSafeMode && cones_count_all_ <= 0) {
        plausability_+= kErrorWeightConesCountAll;
        RCLCPP_ERROR(this->get_logger(), "No cones detected by slam");
    }
}

void CarState::on_timer()
{
    if (kMission!="inspection"){
        this->get_tf_position();
    }
       
    // Estimate vx, vy
    if(kSimulation){
        Vector2d x_est = speed_estimator_.estimate_speed(ax_, r_, delta_, delta_der_, 
            v_front_left_, v_front_right_, v_rear_left_, v_rear_right_, inv_speed_, kSimulation);
        vx_ = x_est(0);
        vy_ = x_est(1); 
    } else {
        vx_ = inv_speed_;
        vy_ = 0.0;
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

    // Check if the mission is finished
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


    // Decrease plausability buffer    
    if (plausability_ > 0){
        plausability_--;
    }


    // Publish run check
    auto run_check_msg = std_msgs::msg::Bool();
    if (kSafeMode){
        run_check_msg.data = (as_status_ == 3) && (plausability_ < kMaxPlausabilityError) && epos_OK_;
    } else {
        run_check_msg.data = (as_status_ == 3);
    }

    run_check_pub_->publish(run_check_msg);

    // Publish steering check
    auto steering_check_msg = std_msgs::msg::Bool();
    steering_check_msg.data = run_check_msg.data && (vx_ >= 0.5);
    steer_check_pub_->publish(steering_check_msg);

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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarState>());
    rclcpp::shutdown();
    return 0;
}

