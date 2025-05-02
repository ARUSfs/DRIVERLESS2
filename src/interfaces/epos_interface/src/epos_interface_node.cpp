/**
 * @file epos_interface_node.cpp
 * @author Francis Rojas (frarojram@gmail.com)
 * @brief EPOS4 controller interface for ARUS Team Driverless pipeline
 */

#include "epos_interface/epos_interface_node.hpp"

bool DEBUG = true;


EPOS_interface::EPOS_interface() : Node("EPOS_interface"), 
    epos_() 
{

    // Declare parameters
    this->declare_parameter<int>("MAX_ACCELERATION", 6000);
    this->declare_parameter<int>("MAX_DECELERATION", 6000);
    this->declare_parameter<int>("PROFILE_VELOCITY", 6000);
    this->declare_parameter<bool>("debug", true);

    // Get parameters
    this->get_parameter("MAX_ACCELERATION", kMaxAcc);
    this->get_parameter("MAX_DECELERATION", kMaxDec);
    this->get_parameter("PROFILE_VELOCITY", kProfileVel);
    this->get_parameter("debug", kDebug);

    // Initialize and connect to EPOS4
    epos_.set_params(kMaxAcc, kMaxDec, kProfileVel);
    epos_.logger_ = this->get_logger();
    epos_.connect_to_device();
    epos_.enable();

    is_shutdown_ = false;

    ext_time_ = this->now().seconds();
    epos_pos_ = 0.0;
    ext_pos_ = 0.0;
    delta_cmd_ = 0.0;
    steer_check_ = false;


    epos_info_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/epos_interface/epos_info", 10);

    cmd_sub_ = this->create_subscription<common_msgs::msg::Cmd>(
        "/controller/cmd", 1, 
        std::bind(&EPOS_interface::cmd_callback, this, std::placeholders::_1));

    steer_check_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/car_state/steer_check", 1, 
        std::bind(&EPOS_interface::steer_check_callback, this, std::placeholders::_1));

    extensometer_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/can_interface/extensometer", 1, 
        std::bind(&EPOS_interface::extensometer_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), 
        std::bind(&EPOS_interface::on_timer, this));
    
    if (kDebug) (this->get_logger(), "EPOS_interface node initialized.");
}


EPOS_interface::~EPOS_interface()
{
    clean_and_close();
}


void EPOS_interface::on_timer()
{
    assert(delta_cmd_ <= M_PI/9.0 && delta_cmd_ >= -M_PI/9.0 && "Angle out of range");
    double epos_cmd = delta_cmd_ - ext_pos_;

    if (!is_shutdown_ && steer_check_){
        epos_.move_to(epos_cmd);
        started_ = true;
    } 
    

    std::vector<double> epos_info = epos_.get_epos_info();
    std_msgs::msg::Float32MultiArray info_msg;
    for (const auto &value : epos_info) {
        info_msg.data.push_back(value);
    }
    // epos_pos_ = epos_info[1];
    epos_info_pub_->publish(info_msg);
    
}


void EPOS_interface::cmd_callback(const common_msgs::msg::Cmd::SharedPtr msg)
{
    delta_cmd_ = msg->delta;
}


void EPOS_interface::extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg){
    // Get smoothed extensometer value until movement starts
    if (started_) return;

    ext_pos_ = 0.8*msg->data + 0.2*ext_pos_;
    ext_time_ = this->now().seconds();

    if (std::abs(ext_pos_) > 22.0*M_PI/180.0){
        if (kDebug) RCLCPP_ERROR(this->get_logger(), "Extensometer value out of range: %f", msg->data);
        rclcpp::shutdown();
        return;
    }
}


void EPOS_interface::steer_check_callback(const std_msgs::msg::Bool::SharedPtr msg){
    steer_check_ = msg -> data;
}


void EPOS_interface::clean_and_close(){
    is_shutdown_ = true;
    epos_.disable();
    epos_.disconnect_device();
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto EPOS_interface_node = std::make_shared<EPOS_interface>();
    rclcpp::spin(EPOS_interface_node);
    rclcpp::shutdown();
    return 0;
}


