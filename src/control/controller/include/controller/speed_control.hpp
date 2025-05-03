/**
 * @file speed_control.h
 * @author Lola Hernandez (lolahercan@gmail.com).
 * @brief Speed control implementation for ARUS Team Driverless pipeline.
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
        double prev_acc_ = 0.0;     // previous acceleration command
        double alpha = 0.7;         // smoothing factor

        const double rho = 1.225;   // air density
        const double CdA = 1.2;     // drag coefficient
        const double Crr = 0.01;    // rolling resistance coefficient
        const double mass = 230;    // mass of the vehicle
        const double g = 9.81;      // gravity
            
        /**
         * @brief Get acceleration command using PID and feedforward control
         */

        double get_acc_command(double target_speed, double target_acc, double vx, double dt) {

            double F_drag = 0.5 * rho * CdA * vx * vx;
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

