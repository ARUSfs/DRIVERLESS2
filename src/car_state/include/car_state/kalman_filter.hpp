#include <rclcpp/rclcpp.hpp>
#include <iostream>
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

        void set_initial_state_and_covariance(VectorXd x, MatrixXd P, rclcpp::Time t){
            if(x.size() != n_) {
                std::cerr << "Initial state vector length is wrong!\nn = " << n_ << std::endl;
            } 
            else {
                x_prev_ = x;
            }

            if(P.cols() != n_ || P.rows() != n_) {
                std::cerr << "Initial covariance matrix dimensions are wrong!\nn = " << n_ << std::endl;

            }
            else {
                P_prev_ = P;
            }

            t_prev_ = t;
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


    private:
        // Kalman Filter atributes
        // Problem size
        int n_; 

        // State and covariance
        VectorXd x_prev_;   // Previous state (n x 1)
        MatrixXd P_prev_;   // Previous covariance (n x n)
        
        VectorXd x_est_;    // Estimated state (n x 1)
        MatrixXd P_est_;    // Estimated covariance (n x n)

        // Process matrices
        MatrixXd A_;        // Transition matrix (n x n) (calculated at each iteration, A = I + dt*M)
        MatrixXd M_;        // Model matrix (n x n) (expresses the evolution of the state over time,
                            // given by first order differential equations)
        MatrixXd B_;        // Control matrix (n x m, where m is the length of the control vector)
        MatrixXd Q_;        // Process covariance matrix (n x n)

        // Measurement matrices
        MatrixXd H_;        // Observation matrix (n x n)
        MatrixXd R_;        // Measurement covariance matrix (n x n)

        // Time 
        rclcpp::Time t_prev_ ;  // Previous time
        double dt_;             // Time interval between iterations
        

};