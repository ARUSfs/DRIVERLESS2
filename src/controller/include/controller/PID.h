#include "rclcpp/rclcpp.hpp" 

double previous_error = 0.0;
double integral = 0.0;


namespace PID {

    double compute_control(double value, double target, double KP, double KI, double KD, double dt) {

        double error = target - value;
        
        integral += error * dt;

        double derivative = (error - previous_error) / dt;
        previous_error = error;

        double control = (error * KP) + (KI * integral) + (KD * derivative) ;

        return control; 
    }

}