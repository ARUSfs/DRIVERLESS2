#include <Eigen/Dense>
using namespace Eigen;

class Estimation
{
    private:
        //Sensor data
        double v_front_right_ = 0;
        double v_front_left_ = 0;
        double v_rear_right_ = 0;
        double v_rear_left_ = 0;
        double ax_ = 0;
        double ay_ = 0;

        //Previous state
        Vector2d x_prev_;   // previous [vx_; vy_]
        Matrix2d P_prev_;   // previous covariance matrix

        //Additional parameters
        Matrix2d Q_;        // process covariance matrix
        Matrix2d R_;        // measurement covariance matrix
        double dt_;         // time step

        //Kalman model matrices
        Matrix2d A_;        // state transition matrix
        Matrix2d B_;        // control matrix
        Matrix2d H_;        // observation matrix


    public:
        Estimation()
        {   
            x_prev_ = Vector2d::Zero();
            P_prev_ = Matrix2d::Zero(); 
            Q_ = 0.1*Matrix2d::Identity();
            R_ = 0.5*Matrix2d::Identity();
            dt_ = 0.1; 

            A_ = Matrix2d::Identity();
            B_ = dt_*Matrix2d::Identity();
            H_ = Matrix2d::Identity();

        }
        
        Estimation(Matrix2d Q, Matrix2d R, double dt)
        {
            x_prev_ = Vector2d::Zero();
            P_prev_ = Matrix2d::Zero();   
            Q_ = Q;
            R_ = R;
            dt_ = dt;

            A_ = Matrix2d::Identity();
            B_ = dt_*Matrix2d::Identity();
            H_ = Matrix2d::Identity();
        }

        void set_measurement_data(double vfr, double vfl, double vrr, double vrl, double ax, double ay)
        {
            v_front_right_ = vfr;
            v_front_left_ = vfl;
            v_rear_right_ = vrr;
            v_rear_left_ = vrl;
            ax_ = ax;
            ay_ = ay;
        }

        Vector2d kalman_velocity_estimation()
        {
            // Control vector
            Vector2d u(ax_,ay_);

            // Prediction
            Vector2d x_pred = A_*x_prev_ + B_*u;
            Matrix2d P_pred = A_*P_prev_*A_.transpose() + Q_;

            // Measurement (wheelspeeds mean)
            double wheelspeed_avg = (v_front_right_ + v_front_left_ + v_rear_right_ + v_rear_left_)/4;
            Vector2d z(wheelspeed_avg, 0);  // measurement vector

            // Kalman gain
            Matrix2d K = ((P_pred*H_.transpose()).array() / (H_*P_pred*H_.transpose() + R_).array()).matrix();

            // Update
            Vector2d x_est = x_pred + K*(z - H_*x_pred);
            Matrix2d P_est = (Matrix2d::Identity() - K*H_)*P_pred;

            // Update estimated values for next iteration
            x_prev_ = x_est;
            P_prev_ = P_est;

            return x_est;
        }
        
};