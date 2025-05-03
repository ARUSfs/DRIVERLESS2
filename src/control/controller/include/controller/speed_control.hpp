/**
 * @file speed_control.h
 * @author Lola Hernandez (lolahercan@gmail.com).
 * @brief Speed control implementation for ARUS Team Driverless pipeline.
 */

#include "controller/PID.hpp"


class SpeedControl
{
    public:
        SpeedControl(){
            pid_= PID();
        }

        PID pid_;
        double prev_acc_ = 0.0;     // previous acceleration command
        double alpha = 0.7;         // smoothing factor

        double rho;     // air density
        double CdA;     // drag coefficient
        double Crr;     // rolling resistance coefficient
        double mass;    // mass of the vehicle
        double g;       // gravity
           
        /**
         * @brief Set vehicle parameters
         */
        void set_params(double rho, double CdA, double Crr, double mass, double g) {
            this->rho = rho;
            this->CdA = CdA;
            this->Crr = Crr;
            this->mass = mass;
            this->g = g;
        }


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

