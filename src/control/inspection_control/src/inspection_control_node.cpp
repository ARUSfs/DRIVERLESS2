#include "inspection/inspection_control_node.hpp"
#include <cmath>
#include <limits>

InspectionControl::InspectionControl() : Node("inspection_control_node"){ 

    this->declare_parameter("KP", 43.87);
    this->declare_parameter("KI", 0.0);
    this->declare_parameter("KD", 0.0);
    this->declare_parameter("AMPLITUDE", 20.0);
    this->declare_parameter("FREQUENCY", 0.2);
    this->declare_parameter("DURATION", 27.0);
    this->get_parameter("KP", KP);
    this->get_parameter("KI", KI);
    this->get_parameter("KD", KD);
    this->get_parameter("AMPLITUDE", kAmplitude);
    this->get_parameter("FREQUENCY", kFrequency);
    this->get_parameter("DURATION", kDuration);
      

    start_time_ = this->get_clock()->now();

    pid_ = PID();    
    pid_.set_params(KP, KI, KD);

    cmd_pub_ = this->create_publisher<common_msgs::msg::Cmd>("/controller/cmd", 10);
    finish_pub_ = this->create_publisher<std_msgs::msg::Float32>("/can_interface/AS_status", 10);

    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
            "/car_state/state", 10, std::bind(&InspectionControl::car_state_callback, this, std::placeholders::_1));
    as_status_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can_interface/AS_status", 10, std::bind(&InspectionControl::as_status_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&InspectionControl::on_timer, this));
}

void InspectionControl::car_state_callback(const common_msgs::msg::State::SharedPtr msg)
{
    vx_ = msg->vx;
}

void InspectionControl::as_status_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    if (msg->data == 3 && as_status_ != 3){ 
        start_time_ = this->get_clock()->now();
    }
    as_status_ = msg->data;
}

void InspectionControl::on_timer()
{   
    if(as_status_ == 3 && this->get_clock()->now().seconds() - start_time_.seconds() < kDuration){
        auto cmd_msg = common_msgs::msg::Cmd();

        // 230 is the maximum toque value
        // Clamp the value between 0% and 20% of the maximum torque
        cmd_msg.acc = std::clamp(pid_.compute_control(vx_, 1.0, 0.01)/230, 0.0, 0.05);

        double t = this->get_clock()->now().seconds() - start_time_.seconds();
        cmd_msg.delta = (kAmplitude*M_PI/180)*sin(2.0*M_PI*kFrequency*t);
        
        cmd_pub_->publish(cmd_msg);

    } else if (as_status_ == 3 && this->get_clock()->now().seconds() - start_time_.seconds() >= kDuration){
        if (vx_ > 0.5){
            auto cmd_msg = common_msgs::msg::Cmd();
            cmd_msg.acc = 0.0;
            cmd_msg.delta = 0.0;
            cmd_pub_->publish(cmd_msg);
        } else {
            auto finish_msg = std_msgs::msg::Float32();
            finish_msg.data = 5;
            finish_pub_->publish(finish_msg);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InspectionControl>());
    rclcpp::shutdown();
    return 0;
}
