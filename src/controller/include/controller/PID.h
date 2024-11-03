#include "rclcpp/rclcpp.hpp" 

rclcpp::Time previous_time;
double previous_error;
double integral;


namespace PID {

    double compute_control(double value, double target, double KP, double KI, double KD, rclcpp::Time current_time) {
        
        double delta_time = (current_time - previous_time).seconds();
        previous_time = current_time;

        double error = target - value;
        integral += error * delta_time;
        double derivative = (error - previous_error) / delta_time;
        previous_error = error;

        double control = (error * KP) + (KI * integral) + (KD * derivative) ;

        return control; 
    }

    void reset(rclcpp::Clock::SharedPtr node_clock) {
        
        previous_error = 0.0;
        integral = 0.0;
        previous_time = node_clock->now();

    }

}