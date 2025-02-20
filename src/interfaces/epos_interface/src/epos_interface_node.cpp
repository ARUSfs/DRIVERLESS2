/**
 * @file epos_interface_node.cpp
 * 
 * @author Francis Rojas (frarojram@gmail.com)
 * 
 * @brief EPOS interface, implementation for ARUS Team Driverless pipeline
 * 
 * @date 22-12-2024
 */

#include "epos_interface/epos_interface_node.hpp"


EPOS_interface::EPOS_interface() : Node("EPOS_interface"), 
    epos_() 
{
    this->declare_parameter<int>("MAX_ACCELERATION", 6000);
    this->declare_parameter<int>("MAX_DECELERATION", 6000);
    this->declare_parameter<int>("PROFILE_VELOCITY", 6000);
    this->get_parameter("MAX_ACCELERATION", MAX_ACC_);
    this->get_parameter("MAX_DECELERATION", MAX_DEC_);
    this->get_parameter("PROFILE_VELOCITY", PROFILE_VEL_);

    epos_.set_params(MAX_ACC_, MAX_DEC_, PROFILE_VEL_);
    epos_.connect_to_device();
    epos_.enable();

    is_shutdown_ = false;

    ext_time_ = this->now().seconds();
    epos_pos_ = 0.0;
    ext_pos_ = 0.0;


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
    
    RCLCPP_INFO(this->get_logger(), "EPOS_interface node initialized.");
}

EPOS_interface::~EPOS_interface()
{
    clean_and_close();
}

void EPOS_interface::cmd_callback(const common_msgs::msg::Cmd::SharedPtr msg)
{
    double angle = msg -> delta * 180 / M_PI;
    assert(angle <= 20.0 && angle >= -20.0 && "Angle out of range");

    if (this->now().seconds() - ext_time_ < 0.2){
        double diff = angle - ext_pos_;
        std::cout << "diff: " << diff << std::endl;

        if (!is_shutdown_ && steer_check_ && std::abs(diff) > 0.1){
            epos_.move_to(epos_pos_ + diff);
        }
    } 
    

    std::vector<double> epos_info = epos_.get_epos_info();
    std_msgs::msg::Float32MultiArray info_msg;
    for (const auto &value : epos_info) {
        info_msg.data.push_back(value);
    }
    epos_pos_ = epos_info[1];
    epos_info_pub_->publish(info_msg);
    
}

void EPOS_interface::extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg){
    ext_pos_ = 0.8*msg->data*180/M_PI + 0.2*ext_pos_;
    ext_time_ = this->now().seconds();
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


