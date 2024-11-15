/**
 * @file PID.hpp
 * 
 * @author Francis Rojas (frarojram@gmail.com).
 * 
 * @brief PID,header and implementation for ARUS Team Driverless pipeline.
 * 
 * @date 15-11-2024
 */

class PID {
public:
    PID(double kp, double ki, double kd)
        : KP_(kp), KI_(ki), KD_(kd), previous_error_(0.0), integral_(0.0) {}
        
    PID() : KP_(0.0), KI_(0.0), KD_(0.0), previous_error_(0.0), integral_(0.0) {}

    /**
     * @brief Calculate acceleration.
     * 
     * @author Team driverless ARUS.
     * 
     * @param  value current value.
     * 
     * @param  target goal value.
     * 
     * @return control for get the goal.
     */
    double compute_control(double value, double target, double dt) {
        double error = target - value;
        integral_ += error * dt;
        double derivative = (error - previous_error_) / dt;
        previous_error_ = error;
        return (KP_ * error) + (KI_ * integral_) + (KD_ * derivative);
    }
    
private:
    const double KP_;
    const double KI_;
    const double KD_;

    double previous_error_;
    double integral_;
};