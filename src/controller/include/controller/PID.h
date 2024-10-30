#include <chrono>

std::chrono::steady_clock::time_point previous_time;
double previous_error;
double integral;
double prev_target;

namespace PID {

    double compute_control(double value, double target, double KP, double KI, double KD, std::chrono::steady_clock::time_point current_time) {
        
        std::chrono::duration<double> delta_time = current_time - previous_time;
        previous_time = current_time;

        double error = target - value;
        integral += error * delta_time.count();
        double derivative = (error - previous_error) / delta_time.count();
        previous_error = error;

        double feed_forward = (target - prev_target) / delta_time.count();
        prev_target = target;

        double control = (error * KP) + (KI * integral) + (KD * derivative) + feed_forward;

        return control; 
    }

    void reset() {
        
        previous_error = 0.0;
        integral = 0.0;
        previous_time = std::chrono::steady_clock::now();
        prev_target = 0.0;
    }

}