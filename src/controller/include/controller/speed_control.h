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
#include <vector>
#include <utility>

double LAD; 
bool is_first_message = true; 
double prev_TS;


namespace SpeedControl {
    PID pid_;
    double KP, KI, KD;
    bool parameters_loaded = false;

    /**
     * @brief Load parameters of controller.
     * 
     * @param node
     */
    void load_parameters(rclcpp::Node::SharedPtr node) {
        if (!parameters_loaded) {
            
            node->declare_parameter("KP", 43.87);
            node->declare_parameter("KI", 1.29);
            node->declare_parameter("KD", 0.0);

            node->get_parameter("KP", KP);
            node->get_parameter("KI", KI);
            node->get_parameter("KD", KD);

            parameters_loaded = true;  
        }
    }

    /**
     * @brief Update speed target
     * 
     * @authors Team driverless ARUS
     * 
     * @param sp
     * @param ap
     * @param vx_
     * @param dt
     * 
     * @details Use the Pure Pursuit libraries to obtain the steering angle, 
     * and use the PID class to process the acceleration control signal.
     * 
     * @return acceleration and steering angle 
    */
    std::pair<double, double> update_target(const float sp, 
            const float ap, 
            double vx_,  
            double dt,
            rclcpp::Node::SharedPtr node
            ) {
        
        load_parameters(node);
        pid_.set_params(KP,KI,KD);

        double acc = 0.0;
        double delta = 0.0;
        LAD = sp / 1.5;

        delta = PurePursuit::get_steering_angle(LAD);
        double control = pid_.compute_control(vx_, sp, dt);
        double feedforward = ap;

        prev_TS = sp;
        acc = control + feedforward;
        acc /= (230*0.2);
    

        return {acc, delta};  

    }

}