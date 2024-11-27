/**
 * @file speed_control.h
 * 
 * @author Francis Rojas (frarojram@gmail.com).
 * 
 * @brief Speed control implementation for ARUS Team Driverless pipeline.
 * 
 * @date 15-11-2024
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
    double get_acc_command(double target_speed, 
                         double target_acc, 
                         double vx,  
                         double dt) 
    {
        double acc;
        double control = pid_.compute_control(vx, target_speed, dt)/(230 * 0.2);
        double feed_forward = target_acc;
        acc = control + feed_forward;
        return acc;
    }

};

