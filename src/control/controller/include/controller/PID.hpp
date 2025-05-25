/**
 * @file PID.hpp
 * @author Lola Hernandez (lolahercan@gmail.com)
 * @brief PID control implementation for ARUS Team Driverless pipeline.
 */

#pragma once

#include <deque>
#include <numeric>


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
         * @brief Compute PID control
         */
        double compute_control(double value, double target, double dt, double target_acc)
        {

            double error = target - value;

            static std::deque<double> error_history;
            const size_t window_size = 10;
            error_history.push_back(error);

            if (error_history.size() > window_size){
                error_history.pop_front();
            }

            double smoothed_error = std::accumulate(error_history.begin(), error_history.end(), 0.0) / error_history.size();   

            integral_ += smoothed_error * dt;

            double derivative = (smoothed_error - previous_error_) / dt;
            previous_error_ = smoothed_error;


            double KP_var = KP_ + std::abs(target_acc / 15.0);

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

            // if the error is greater than 5, we use the full PID controller
            if (value > 5) {
                return (KP_var * smoothed_error) + (KI_ * integral_) + (KD_ * derivative);
            } else {
                return (KP_var * smoothed_error);
            }

        }

    private:
        double KP_;
        double KI_;
        double KD_;

        double previous_error_;
        double integral_;
};
