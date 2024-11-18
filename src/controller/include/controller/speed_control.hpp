/**
 * @file speed_control.h
 * 
 * @author Francis Rojas (frarojram@gmail.com).
 * 
 * @brief Speed control implementation for ARUS Team Driverless pipeline.
 * 
 * @date 15-11-2024
 */

#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include "common_msgs/msg/trajectory.hpp"
#include "controller/PID.hpp"
#include "rclcpp/rclcpp.hpp"
#include <utility>

class SpeedControl {
public:
    SpeedControl() 
    : pid_(), 
      prev_TS_(0.0) {}

    PID& get_pid() {return pid_;}
          
    /**
     * @brief Update speed target
     * 
     * @param sp Target speed.
     * @param ap Feedforward acceleration parameter.
     * @param vx_ Current velocity.
     * @param dt Time delta.
     * 
     * @return A pair containing the acceleration.
     */
    double update_target(const float sp, 
                         const float ap, 
                         double vx_,  
                         double dt) {

        double acc;
        double control = pid_.compute_control(vx_, sp, dt);
        double feedforward = ap;

        prev_TS_ = sp;
        acc = control + feedforward;
        acc /= (230 * 0.2);

        return acc;
    }

private:
    PID pid_;
    double prev_TS_;
};

#endif // SPEED_CONTROL_H

