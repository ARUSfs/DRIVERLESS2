#include "rclcpp/rclcpp.hpp" 

double previous_error;
double integral;


namespace PID {

    double compute_control(double value, double target, double KP, double KI, double KD, double delta_time) {

        double error = target - value;
        
        integral += error * delta_time;

        double derivative = (error - previous_error) / delta_time;
        previous_error = error;

        double control = (error * KP) + (KI * integral) + (KD * derivative) ;

        return control; 
    }

    void reset() {
        previous_error = 0.0;
        integral = 0.0;
    }

}