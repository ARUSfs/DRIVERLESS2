#include "inspection/inspection_control_node.hpp"
#include <cmath>

InspectionControl::InspectionControl() : Node("inspection_control_node"){   

    start_time_ = this->get_clock()->now();

    speed_control_ = SpeedControl();    
    speed_control_.pid_.set_params(43.87,1.29,0.0);

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
    double DURATION = 27.0;
    // double AMPLITUDE = 20*M_PI/180;
    // double FREQUENCY = 0.2;
    double AMPLITUDE = 20*M_PI/180;
    double FREQUENCY = 0.2;


    if(as_status_ == 0x02 && this->get_clock()->now().seconds() - start_time_.seconds() < DURATION){
        auto cmd_msg = common_msgs::msg::Cmd();
        cmd_msg.acc = speed_control_.get_acc_command(2.0, 0.0, vx_, 0.1);

        double t = this->get_clock()->now().seconds() - start_time_.seconds();
        cmd_msg.delta = AMPLITUDE*sin(2.0*M_PI*FREQUENCY*t);
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