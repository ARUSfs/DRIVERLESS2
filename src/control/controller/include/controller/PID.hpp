#ifndef PID_HPP
#define PID_HPP

/**
 * @file PID.hpp
 *
 * @author Lola Hernandez (lolahercan@gmail.com).
 *
 * @brief PID,header and implementation for ARUS Team Driverless pipeline.
 *
 * @date 11-02-2025
 */
class PID
{
public:
    PID()
    {
        KP_ = 0.0;
        KI_ = 0.0;
        KD_ = 0.0;
        previous_error_ = 0.0;
        integral_ = 0.0;
    }

    void set_params(double KP, double KI, double KD)
    {
        KP_ = KP;
        KI_ = KI;
        KD_ = KD;
    }

    /**
     * @brief Calculate acceleration.
     * @author Team driverless ARUS.
     * @param  value current value.
     * @param  target goal value.
     * @param dt delta time.
     * @param target_acc target acceleration
     * @return control for get the goal.
     */
    double compute_control(double value, double target, double dt, double target_acc)
    {

        double error = target - value;

        integral_ += error * dt;

        double derivative = (error - previous_error_) / dt;

        double KP_var = 0.0;
        KP_var = KP_ + std::abs(target_acc / 15.0);

        const double max_integral = 5.0;
        const double min_integral = -5.0;

        if (integral_ > max_integral)
        {
            integral_ = max_integral;
        }
        else if (integral_ < min_integral)
        {
            integral_ = min_integral;
        }

        previous_error_ = error;

        if (value > 5)
        {
            return (KP_var * error) + (KI_ * integral_) + (KD_ * derivative);
        }

        return (KP_var * error);
    }

private:
    double KP_;
    double KI_;
    double KD_;

    double previous_error_;
    double integral_;
};

#endif