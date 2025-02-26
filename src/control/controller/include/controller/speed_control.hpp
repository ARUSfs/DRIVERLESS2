/**
 * @file speed_control.h
 * 
 * @author Lola Hernandez (lolahercan@gmail.com).
 * 
 * @brief Speed control implementation for ARUS Team Driverless pipeline.
 * 
 * @date 11-02-2025
 */
#include "common_msgs/msg/trajectory.hpp"
#include "controller/PID.hpp"
#include "rclcpp/rclcpp.hpp"
#include <utility>

class SpeedControl
{
public:
    SpeedControl(){
        pid_= PID();
    }

    PID pid_;
    double prev_acc_ = 0.0;
    double alpha = 0.5; // smoothing factor

    const double rho = 1.225;
    const double Cd = 0.3;
    const double A = 2.0;
    const double Crr = 0.01;
    const double mass = 230;
    const double g = 9.81;
          
    /**
     * @brief Get acceleration command.
     * 
     * @param target_speed Speed profile target.
     * @param target_acc Acceleration profile target.
     * @param vx Current velocity.
     * @param dt Time delta.
     * 
     * @return A pair containing the acceleration.
     */

    double get_acc_command(double target_speed, double target_acc, double vx, double dt) {

        double F_drag = 0.5 * rho * Cd * A * vx * vx;
        double F_roll = Crr * mass * g;
        double a_loss = (F_drag + F_roll) / mass;

        double control = pid_.compute_control(vx, target_speed, dt, target_acc);

        double feed_forward = target_acc + a_loss;
        double acc = control + feed_forward;

        // Smooth the acceleration command using exponential moving average
        double smoothed_acc = alpha * acc + (1.0 - alpha) * prev_acc_;
        prev_acc_ = smoothed_acc;

        return smoothed_acc;
    }

};

