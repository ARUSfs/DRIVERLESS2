#include "inspection/inspection_control_node.hpp"
#include <cmath>

InspectionControl::InspectionControl() : Node("inspection_control_node"){ 

    this->declare_parameter("KP", 43.87);
    this->declare_parameter("KI", 1.29);
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

    speed_control_ = SpeedControl();    
    speed_control_.pid_.set_params(KP, KI, KD);

    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
            "/car_state/state", 10, std::bind(&InspectionControl::car_state_callback, this, std::placeholders::_1));
    as_status_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/can/AS_status", 10, std::bind(&InspectionControl::as_status_callback, this, std::placeholders::_1));
    cmd_publisher_ = this->create_publisher<common_msgs::msg::Cmd>("/controller/cmd", 10);
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&InspectionControl::on_timer, this));
}

void InspectionControl::car_state_callback(const common_msgs::msg::State::SharedPtr msg)
{
    vx_ = msg->vx;
}

void InspectionControl::as_status_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    if (msg->data == 0x02 && as_status_ != 0x02){
        as_status_ = msg->data;
        start_time_ = this->get_clock()->now();
    }
}

void InspectionControl::on_timer()
{   
    if(as_status_ == 0x02 && this->get_clock()->now().seconds() - start_time_.seconds() < kDuration){
        auto cmd_msg = common_msgs::msg::Cmd();
        cmd_msg.acc = speed_control_.get_acc_command(2.0, 0.0, vx_, 0.1);

        double t = this->get_clock()->now().seconds() - start_time_.seconds();
        cmd_msg.delta = (kAmplitude*M_PI/180)*sin(2.0*M_PI*kFrequency*t);
        cmd_publisher_->publish(cmd_msg);
    }
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InspectionControl>());
    rclcpp::shutdown();
    return 0;
}