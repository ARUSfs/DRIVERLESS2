/**
 * @file kalman_filter.hpp
 * @author José Manuel Landero (josemlandero05@gmail.com).
 * @brief KalmanFilter class header for ARUS Team Driverless pipeline.
 * Contains the class definition and implementation of all necessary methods for the Kalman filter.
 * @date 03-01-2025
 */
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>
using namespace Eigen;

/**
 * @brief Class containing a generic Kalman filter.
 * Allows you to initialize all of the filter atributes and perform iterative state estimation
 * (only for linear models).
 */
class KalmanFilter
{
    private:
        // Kalman Filter atributes
        // Problem size
        int n_;             // Number of state variables
        int m_;             // Control vector length
        int p_;             // Measurement vector length

        // Time at previous estimation
        std::chrono::steady_clock::time_point t_ ;  

        // Estimated state and covariance
        VectorXd x_;        // Previous state vector (n x 1)
        MatrixXd P_;        // Previous state covariance matrix (n x n)

        // Process matrices
        MatrixXd M_;        // Model matrix (n x n) 
        MatrixXd B_;        // Control matrix (n x m)
        VectorXd u_;        // Previous control vector (m x 1)
        // M and B express the evolution of the state over time, given by first order differential equations
        // x(k+1) = x(k) + dt*(M * x(k) + B * u(k))
        MatrixXd Q_;        // Process covariance matrix (n x n)

        // Measurement matrices
        MatrixXd H_;        // Observation matrix (p x n)
        MatrixXd R_;        // Measurement covariance matrix (p x p)

    public:
        /**
         * @brief Construct a new KalmanFilter object
         * It just creates an empty KalmanFilter object, with no default values.
         */
        KalmanFilter(){

        };

        /**
         * @brief Problem size initializer
         * Initializes n, m and p.
         * 
         * @param n Number of state variables
         * @param m Control vector length
         * @param p Measurement vector length
         */
        void set_problem_size(int n, int m, int p){
            n_ = n;
            m_ = m;
            p_ = p;
        }

        /**
         * @brief Initial data initializer
         * Initializes x, P, u and t for the first iteration, checking that dimensions are correct.
         * 
         * @param x Initial state
         * @param P Initial state covariance
         * @param u Initial control vector
         */
        void set_initial_data(VectorXd x, MatrixXd P, VectorXd u){
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

            if(u.size() != m_){
                std::cerr << "Initial control vector lenght is wrong!\nm = " << m_ << std::endl;
            }
            else {
                u_ = u;
            }

            t_ = std::chrono::steady_clock::now();
        }

        /**
         * @brief Process matrices initializer
         * Initializes process matrices M, B and Q, checking that dimensions are correct.
         * 
         * @param M Model matrix
         * @param B Control matrix
         * @param Q Process covariance matrix
         */
        void set_process_matrices(MatrixXd M, MatrixXd B, MatrixXd Q){
            if(M.cols() != n_ || M.rows() != n_) {
                std::cerr << "Model matrix dimensions are wrong!\nn = " << n_ << std::endl;
            }
            else {
                M_ = M;
            }

            if(B.rows() != n_ || B.cols() != m_) {
                std::cerr << "Control matrix dimensions are wrong!\nn = " << n_ << ", m = " << m_ << std::endl;
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

        /**
         * @brief Measurement matrices initializer
         * Initializes measurement matrices H and R, checking that dimensions are correct.
         * 
         * @param H Observation matrix
         * @param R Measurement covariance matrix
         */
        void set_measurement_matrices(MatrixXd H, MatrixXd R){
            if(H.cols() != p_ || H.rows() != n_) {
                std::cerr << "Observation matrix dimensions are wrong!\np= " << p_ <<", n = " << n_ << std::endl;
            }
            else {
                H_ = H;
            }

            if(R.cols() != p_ || R.rows() != p_) {
                std::cerr << "Measurement covariance matrix dimensions are wrong!\np = " << p_ << std::endl;
            }
            else {
                R_ = R;
            }
        }

        /**
         * @brief Model matrix updater
         * Updates model matrix M for this iteration.
         * 
         * @param M Model matrix
         */
        void update_model_matrix(MatrixXd M){
            if(M.cols() != n_ || M.rows() != n_) {
                std::cerr << "Model matrix dimensions are wrong!\nn = " << n_ << std::endl;
            }
            else {
                M_ = M;
            }
        }

        /**
         * @brief State estimator
         * Performs the Kalman filter process to estimate the current state and saves estimated data
         * for the next iteration.
         * 
         * @param u Current control vector 
         * @param z Current state measurement
         */
        void estimate_state(VectorXd u, VectorXd z){
            // Check data length
            if(u.size() != m_) {
                std::cerr << "Control vector lenght is wrong!\nm = " << m_ << std::endl;
            }

            if(z.size() != p_) {
                std::cerr << "Measurement vector length is wrong!\np= " << p_ << std::endl;
            }
            

            // PREDICTION STAGE
            // Transition matrix 
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(now - t_).count() * pow(10, -9);

            MatrixXd A(n_, n_);
            A = MatrixXd::Identity(n_, n_) + dt * M_; 

            // State prediction
            VectorXd x_pred = A * x_ + dt * B_ * u_;
            
            // Covariance prediction
            MatrixXd P_pred = A * P_ * A.transpose() + Q_;

            // INNOVATION
            VectorXd v = z - H_ * x_pred;

            // KALMAN GAIN
            MatrixXd K = P_pred * H_.transpose() * (H_ * P_pred * H_.transpose() + R_).inverse();

            // UPDATE STAGE 
            // State estimation
            VectorXd x_est = x_pred + K * v;

            // Covariance estimation
            MatrixXd P_est = (MatrixXd::Identity(n_, n_) - K * H_) * P_pred;

            // Save estimated data for the next iteration
            t_ = std::chrono::steady_clock::now();
            x_ = x_est;
            P_ = P_est;
            u_ = u;
        }

        /**
         * @brief Estimated state getter
         * Returns last estimated state vector.
         * 
         * @return VectorXd
         */
        VectorXd get_estimated_state(){
            return x_;
        }

        /**
         * @brief Estimated state covariance getter
         * Returns last estimated state covariance matrix.
         * 
         * @return MatrixXd
         */
        MatrixXd get_estimated_covariance(){
            return P_;
        }

};