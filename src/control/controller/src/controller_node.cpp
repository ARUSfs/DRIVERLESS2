/**
 * @file controller_node.cpp
 * @author Francis Rojas (frarojram@gmail.com)
 * @brief Controller node implementation for ARUS Team Driverless pipeline
 */

#include "controller/controller_node.hpp"


Controller::Controller() : Node("controller"),  
    speed_control_(),
    pure_pursuit_(),
    lti_mpc_()
{

    // Topic
    this->declare_parameter<std::string>("state_topic", "/car_state/state");
    this->declare_parameter<std::string>("run_check_topic", "/car_state/run_check");
    this->declare_parameter<std::string>("trajectory_topic", "/path_planning/trajectory");
    this->declare_parameter<std::string>("optimized_trajectory_topic", "/trajectory_optimization/trajetory");
    this->declare_parameter<std::string>("cmd_topic", "/controller/cmd");
    this->declare_parameter<std::string>("pursuit_point_topic", "/controller/pursuit_point");
    this->declare_parameter<std::string>("target_speed_topic", "/controller/target_speed");
    this->declare_parameter<std::string>("braking_procedure_topic", "/car_state/braking_procedure");
    this->get_parameter("state_topic", kStateTopic);
    this->get_parameter("run_check_topic", kRunCheckTopic);
    this->get_parameter("trajectory_topic", kTrajectoryTopic);
    this->get_parameter("optimized_trajectory_topic", kOptimizedTrajectoryTopic);
    this->get_parameter("cmd_topic", kCmdTopic);
    this->get_parameter("pursuit_point_topic",kPursuitPointTopic);
    this->get_parameter("target_speed_topic", kTargetSpeedTopic);
    this->get_parameter("braking_procedure_topic", kBrakingProcedureTopic);

    // Parameters
    this->declare_parameter<std::string>("first_lap_steer_control", "PP");
    this->declare_parameter<std::string>("optimized_steer_control", "PP");
    this->declare_parameter<double>("speed_timer_frequency", 100.0);
    this->declare_parameter<double>("steer_timer_frequency", 100.0);
    this->declare_parameter<bool>("use_optimized_trajectory", false);
    this->get_parameter("first_lap_steer_control", kFirstLapSteerControl);
    this->get_parameter("optimized_steer_control", kOptimizedSteerControl);
    this->get_parameter("speed_timer_frequency", kSpeedTimerFreq);
    this->get_parameter("steer_timer_frequency", kSteerTimerFreq);
    this->get_parameter("use_optimized_trajectory", kUseOptimizedTrajectory);


    // Pure-Pursuit
    this->declare_parameter<double>("look_ahead_distance", 4.0);
    this->declare_parameter<double>("optimized_look_ahead_distance", 7.0);
    this->get_parameter("look_ahead_distance", kLAD);
    this->get_parameter("optimized_look_ahead_distance", kOptLAD);

    // PID
    this->declare_parameter<double>("target", 8.0);
    this->declare_parameter<double>("braking_decc", 3.0);
    this->declare_parameter<double>("KP", 43.87);
    this->declare_parameter<double>("KI", 1.29);
    this->declare_parameter<double>("KD", 0.0); 
    this->get_parameter("target", kTargetSpeed);
    this->get_parameter("braking_decc", kBrakingDecc);
    this->get_parameter("KP", KP);
    this->get_parameter("KI", KI);
    this->get_parameter("KD", KD);

    // MPC
    this->declare_parameter<double>("cost_lateral_error", 10.0);
    this->declare_parameter<double>("cost_angular_error", 5.0);
    this->declare_parameter<double>("cost_steering_delta", 1000.0); 
    this->declare_parameter<int>("compensation_steps", 5); 
    this->declare_parameter<double>("cornering_stiffness_front", -25000.0); 
    this->declare_parameter<double>("cornering_stiffness_rear", -20000.0); 
    this->declare_parameter<int>("prediction_horizon", 65); 
    this->declare_parameter<double>("ts_mpc", 0.02);
    this->declare_parameter<double>("steer_model_u", 307.0); 
    this->declare_parameter<double>("steer_model_delta", -306.3); 
    this->declare_parameter<double>("steer_model_delta_v", -25.69); 

    this->get_parameter("cost_lateral_error", kCostLateralDeviation);
    this->get_parameter("cost_angular_error", kCostAngularDeviation);
    this->get_parameter("cost_steering_delta", kCostSteeringDelta);   
    this->get_parameter("compensation_steps", kCompensationSteps); 
    this->get_parameter("cornering_stiffness_front", kCorneringStiffnessF); 
    this->get_parameter("cornering_stiffness_rear", kCorneringStiffnessR); 
    this->get_parameter("prediction_horizon", kPredictionHorizon); 
    this->get_parameter("ts_mpc", kTsMPC);
    this->get_parameter("steer_model_u", kSteerModelU); 
    this->get_parameter("steer_model_delta", kSteerModelDelta); 
    this->get_parameter("steer_model_delta_v", kSteerModelDeltaV); 

    // Cmd limits
    this->declare_parameter<double>("min_cmd", -3.0);
    this->declare_parameter<double>("max_cmd", 5.0);
    this->declare_parameter<double>("max_steer", 20.0);
    this->get_parameter("min_cmd", kMinCmd);
    this->get_parameter("max_cmd", kMaxCmd);
    this->get_parameter("max_steer", kMaxSteer);

    // Vehicle parameters
    this->declare_parameter<double>("wheel_base", 1.535);
    this->declare_parameter<double>("rho", 1.225);
    this->declare_parameter<double>("CdA", 1.2);
    this->declare_parameter<double>("Crr", 0.01);
    this->declare_parameter<double>("mass", 230);
    this->declare_parameter<double>("g", 9.81);
    this->declare_parameter<double>("Izz", 180);
    this->declare_parameter<double>("weight_dist_rear", 0.5);
    this->get_parameter("wheel_base", kWheelBase);
    this->get_parameter("rho", kRho);
    this->get_parameter("CdA", kCdA);
    this->get_parameter("Crr", kCrr);
    this->get_parameter("mass", kMass);
    this->get_parameter("g", kG);
    this->get_parameter("Izz", kIzz);
    this->get_parameter("weight_dist_rear", kWeightDistributionRear);

    this->declare_parameter<bool>("debug", true);
    this->get_parameter("debug", kDebug);
    

    speed_control_.set_params(kRho, kCdA, kCrr, kMass, kG);
    speed_control_.pid_.set_params(KP,KI,KD);
    pure_pursuit_.wheel_base_ = kWheelBase;
    lti_mpc_.set_params(kCostLateralDeviation,kCostAngularDeviation,kCostSteeringDelta,kCompensationSteps,
            kPredictionHorizon, kTsMPC, kCorneringStiffnessF, kCorneringStiffnessR, kWheelBase, kWeightDistributionRear,
            kMass, kIzz, kSteerModelU, kSteerModelDelta, kSteerModelDeltaV);

    previous_time_ = this->get_clock()->now();

    // Timers
    speed_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / kSpeedTimerFreq)),
        std::bind(&Controller::on_speed_timer, this));

    steer_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / kSteerTimerFreq)),
        std::bind(&Controller::on_steer_timer, this));

    // Subscribers
    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
        kStateTopic, 1, std::bind(&Controller::car_state_callback, this, std::placeholders::_1));
    run_check_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        kRunCheckTopic, 1, std::bind(&Controller::run_check_callback, this, std::placeholders::_1));
    trayectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kTrajectoryTopic, 1, std::bind(&Controller::trajectory_callback, this, std::placeholders::_1));
    optimized_trajectory_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        kOptimizedTrajectoryTopic, 1, std::bind(&Controller::optimized_trajectory_callback, this, std::placeholders::_1));
    braking_procedure_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        kBrakingProcedureTopic, 1, std::bind(&Controller::braking_procedure_callback, this, std::placeholders::_1));

    // Publishers
    cmd_pub_ = this->create_publisher<common_msgs::msg::Cmd>(kCmdTopic, 10);
    pursuit_point_pub_ = this->create_publisher<common_msgs::msg::PointXY>(kPursuitPointTopic, 10);
    target_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(kTargetSpeedTopic, 10);
}


void Controller::on_speed_timer()
{
    if(!(pointsXY_.empty()) && run_check_){
        get_global_index();

        if (braking_procedure_){
            target_speed_ = std::max(0.0, std::sqrt(vx_*vx_ - 2*kBrakingDecc*target_speed_));
        } else if(!(speed_profile_.empty())){
            target_speed_ = speed_profile_.at(index_global_);
        } else{
            target_speed_ = kTargetSpeed;
        }
        std_msgs::msg::Float32 target_speed_msg;
        target_speed_msg.data = target_speed_;
        target_speed_pub_->publish(target_speed_msg);

        double target_acc = 0.0;
        if(!(acc_profile_.empty())){
            target_acc = acc_profile_.at(index_global_);
        }
        

        double dt = (this->now() - previous_time_).seconds();
        acc_cmd_ = speed_control_.get_acc_command(target_speed_, target_acc, vx_, dt);
        previous_time_ = this->now();

        common_msgs::msg::Cmd cmd;       
        cmd.acc = std::clamp(acc_cmd_, kMinCmd, kMaxCmd);
        if (vx_ < 3) cmd.acc = std::clamp(acc_cmd_, 0.0, kMaxCmd);
        cmd.delta = std::clamp(delta_cmd_, -kMaxSteer*M_PI/180, kMaxSteer*M_PI/180);
        cmd_pub_ -> publish(cmd); 
    }
}


void Controller::on_steer_timer()
{
    if ((!optimized_ && kFirstLapSteerControl=="PP") || (optimized_ && kOptimizedSteerControl=="PP")){
        pure_pursuit_.set_path(pointsXY_);
        pure_pursuit_.set_position(x_, y_, yaw_);

        if (!optimized_) {
            pure_pursuit_.get_steering_angle(index_global_, kLAD);
        } else {
            pure_pursuit_.get_steering_angle(index_global_, kOptLAD);
        }
        delta_cmd_ = pure_pursuit_.delta_cmd_;
        Point pursuit_point = pure_pursuit_.pursuit_point_;

        common_msgs::msg::PointXY pursuit_point_msg;
        pursuit_point_msg.x = pursuit_point.x;
        pursuit_point_msg.y = pursuit_point.y;
        pursuit_point_pub_ -> publish(pursuit_point_msg);

    } else if ((!optimized_ && kFirstLapSteerControl=="MPC") || (optimized_ && kOptimizedSteerControl=="MPC"))
    {
        Point position;
        position.x = x_;
        position.y = y_;
        if (!(s_.empty())){
            double t0 = this->now().seconds();

            lti_mpc_.set_reference_trajectory(pointsXY_, s_, position, yaw_, vx_, index_global_);
            delta_cmd_ = lti_mpc_.calculate_control(delta_, v_delta_, vy_, r_);

            if (kDebug) RCLCPP_INFO(this->get_logger(), "MPC time: %f ms", (this->now().seconds() - t0)*1000);
        }
    }
}


void Controller::car_state_callback(const common_msgs::msg::State::SharedPtr msg)
{
    x_ = msg -> x;
    y_ = msg -> y;
    yaw_ = msg -> yaw;
    vx_ = msg -> vx;
    vy_ = msg -> vy;
    r_ = msg -> r;
    delta_ = msg -> delta;

    v_delta_ = 0.7*v_delta_ + 0.3*(delta_ - prev_delta_)/0.01;
    prev_delta_ = delta_;

}


void Controller::optimized_trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
{   
    if (!kUseOptimizedTrajectory){
        return;
    }

    trajectory_callback(msg);
    new_trajectory_ = true;
    optimized_ = true;
}


void Controller::trajectory_callback(const common_msgs::msg::Trajectory::SharedPtr msg)
{
    if (optimized_ || msg->points.size() < 3){
        return;
    }

    // Check if the trajectory is the same
    if(msg->points.size() != pointsXY_.size()
       || msg->points[msg->points.size()-1].x != pointsXY_[pointsXY_.size()-1].x
       || msg->points[msg->points.size()-1].y != pointsXY_[pointsXY_.size()-1].y ){
        new_trajectory_ = true;

        pointsXY_.clear();
    
        std::vector<common_msgs::msg::PointXY> points_common = msg -> points;
        for (const auto &pointXY : points_common) {
            Point point;
            point.x = pointXY.x;
            point.y = pointXY.y;
            pointsXY_.push_back(point);
        }
        s_= msg -> s;
        k_ = msg -> k;
        speed_profile_ = msg -> speed_profile;
        acc_profile_ = msg -> acc_profile; 
    }
}


void Controller::run_check_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    run_check_ = msg->data;
}


void Controller::braking_procedure_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    braking_procedure_ = msg->data;
}


void Controller::get_global_index() {
    double min_dist = std::numeric_limits<double>::max();
    int N = pointsXY_.size();

    // If the trajectory is not updated, the search starts from the last index
    int i;
    if(new_trajectory_){
        i = 0; 
    } else {
        i = index_global_; 
    }

    // Search for the closest point from the current position
    // N*1.1 and i%N is added to allow restarting loops (trackdrive)
    while (i < int(N*1.1)) {
        double dx = pointsXY_[i % N].x - x_;
        double dy = pointsXY_[i % N].y - y_;
        double dist = dx * dx + dy * dy;

        if (dist < min_dist) {
            min_dist = dist;
            index_global_ = i % N;
        } else if(!new_trajectory_ && dist > 5.0){ 
            // If the distance is greater than 5 meters from last index, 
            // the search is stopped to avoid errors due to loops in the trajectory (skidpad)
            break;
        }
        i++;
    }
    new_trajectory_ = false;
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<Controller>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}




