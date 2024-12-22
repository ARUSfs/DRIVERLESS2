#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;

class KalmanFilter
{
    public:
        KalmanFilter()
        {

        };


    private:
        // Kalman Filter atributes
        // Problem size
        int n_; 

        // State and covariance
        VectorXd x_prev_;   // Previous state 
        MatrixXd P_prev_;   // Previous covariance
        
        VectorXd x_est_;    // Estimated state and covariance
        MatrixXd P_est_;    // Estimated covariance

        // Process matrices
        MatrixXd A_;        // Transition matrix
        MatrixXd M_;        // Model matrix (relation between variables 
                            // given by first order differential equations)
        MatrixXd B_;        // Control matrix
        MatrixXd Q_;        // Process covariance matrix

        // Measurement matrices
        MatrixXd H_;        // Observation matrix
        MatrixXd R_;        // Measurement noise

        // Time 
        rclcpp::Time t_prev_ ;
        double dt_;
        

};