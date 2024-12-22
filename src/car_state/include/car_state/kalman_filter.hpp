#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>
using namespace Eigen;

class KalmanFilter
{
    public:
        KalmanFilter(){

        };

        void set_problem_size(int n){
            n_ = n;
        }

        void set_initial_state_and_covariance(VectorXd x, MatrixXd P){
            if(x.size() != n_) {
                std::cerr << "Initial state vector length is wrong!\nn = " << n_ << std::endl;
            } 
            else {
                x_ = x;
            }

            if(P.cols() != n_ || P.rows() != n_) {
                std::cerr << "Initial covariance matrix dimensions are wrong!\nn = " << n_ << std::endl;

            }
            else {
                P_ = P;
            }

            t_ = std::chrono::steady_clock::now();
        }

        void set_process_matrices(MatrixXd M, MatrixXd B, MatrixXd Q){
            if(M.cols() != n_ || M.rows() != n_) {
                std::cerr << "Model matrix dimensions are wrong!\nn = " << n_ << std::endl;
            }
            else {
                M_ = M;
            }

            if(B.rows() != n_) {
                std::cerr << "Control matrix dimensions are wrong!\nn = " << n_ << std::endl;
            }
            else {
                B_ = B;
            }

            if(Q.cols() != n_ || Q.rows() != n_) {
                std::cerr << "Process covariance matrix dimensions are wrong!\nn = " << n_ << std::endl;
            }
            else {
                Q_ = Q;
            }
        }

        void set_measurement_matrices(MatrixXd H, MatrixXd R){
            if(H.cols() != n_ || H.rows() != n_) {
                std::cerr << "Observation matrix dimensions are wrong!\nn = " << n_ << std::endl;
            }
            else {
                H_ = H;
            }

            if(R.cols() != n_ || R.rows() != n_) {
                std::cerr << "Measurement covariance matrix dimensions are wrong!\nn = " << n_ << std::endl;
            }
            else {
                R_ = R;
            }
        }

        void estimate_state(VectorXd u, VectorXd z){
            // PREDICTION STAGE
            // Transition matrix 
            MatrixXd A(n_, n_);
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(now - t_).count() * pow(10, -9);
            A = MatrixXd::Identity(n_, n_) + dt*M_; 

            // State prediction
            VectorXd x_pred = A * x_ + dt*B_ * u;
            
            // Covariance prediction
            MatrixXd P_pred = A * P_ * A.transpose() + Q_;

            // INNOVATION
            VectorXd v = z - H_ * x_pred;

            // KALMAN GAIN
            MatrixXd K = P_pred * H_.transpose() * (H_ * P_pred * H_.transpose() + R_).inverse();

            // ESTIMATION STAGE 
            // State estimation
            VectorXd x_est = x_pred + K * v;

            // Covariance estimation
            MatrixXd P_est = (MatrixXd::Identity(n_, n_) - K * H_) * P_pred;

            // Save estimated data for the next iteration
            t_ = std::chrono::steady_clock::now();
            x_ = x_est;
            P_ = P_est;
        }

        VectorXd get_estimated_state(){
            return x_;
        }

        MatrixXd get_estimated_covariance(){
            return P_;
        }



    private:
        // Kalman Filter atributes
        // Problem size
        int n_; 

        // Time at previous estimation
        std::chrono::steady_clock::time_point t_ ;  

        // State and covariance
        VectorXd x_;        // State vector (n x 1)
        MatrixXd P_;        // Covariance matrix (n x n)

        // Process matrices
        MatrixXd M_;        // Model matrix (n x n) 
        MatrixXd B_;        // Control matrix (n x m, where m is the length of the control vector u)
        // M and B express the evolution of the state over time, given by first order differential equations
        // x(k+1) = x(k) + dt*(M * x(k) + B * u)
        MatrixXd Q_;        // Process covariance matrix (n x n)

        // Measurement matrices
        MatrixXd H_;        // Observation matrix (n x n)
        MatrixXd R_;        // Measurement covariance matrix (n x n)

        
        

};