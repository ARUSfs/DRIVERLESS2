/**
 * @file speed_estimator.hpp
 * @author José Manuel Landero (josemlandero05@gmail.com).
 * @brief SpeedEstimator class header for ARUS Team Driverless pipeline.
 * Contains combined instances of KalmanFilter class to estimate car's speed.
 * @date 07-03-2025
 */
#include "kalman_filter.hpp"

class SpeedEstimator 
{
private:
    // Estimation filters
    KalmanFilter v_filter_kin_;
    KalmanFilter v_filter_dyn_;

    // Bicycle model variables
    double L_ = 1.5333;
    double mass_distr_R = 0.5;
    double lf_ = mass_distr_R * L_;
    double lr_ = (1-mass_distr_R) * L_;

    double vx_blend_min_ = 5.5;
    double vx_blend_max_= 9.5;

    double B_lat_ = 9.0;
    double C_lat_ = 1.54;
    double D_lat_ = -1.537;
    double cf_ = 26000;
    double cr_ = 22000;
    double m_ = 270.0;

    // Current speed
    double vx_ = 0;
    double vy_ = 0;

    // State and measurement dimensions
    int n_ = 2;
    int p_ = 2;  
    
    // Initial state and covariance
    VectorXd x_initial_ = VectorXd::Zero(n_);
    MatrixXd P_initial_ = 0.1 * MatrixXd::Identity(n_,n_);

    // Observation matrix
    MatrixXd H_ = MatrixXd::Identity(p_,n_);

    // Kinematic model variables
    int m_kin_ = 2;
    VectorXd u_initial_kin_ = VectorXd::Zero(m_kin_);

    MatrixXd M_kin_ = MatrixXd::Zero(n_,n_);
    MatrixXd B_kin_ = MatrixXd::Zero(n_,m_kin_);

    MatrixXd Q_kin_ = Vector2d(0.1, 0.05).asDiagonal();
    MatrixXd R_kin_ = Vector2d(0.125, 0.075).asDiagonal();

    // Dynamic model variables
    int m_dyn_ = 3;
    VectorXd u_initial_dyn_ = VectorXd::Zero(m_dyn_);

    MatrixXd M_dyn_ = MatrixXd::Zero(n_,n_);
    MatrixXd B_dyn_ = MatrixXd::Zero(n_,m_dyn_);

    MatrixXd Q_dyn_ = Vector2d(0.125, 0.1).asDiagonal();
    MatrixXd R_dyn_ = Vector2d(0.175, 0.15).asDiagonal();


public:
    /**
     * @brief Speed Estimator initializer
     * Initializes both Kinematic and Dynamic Bicycle kalman filters.
     */
    void initialize_speed_estimator(){
        // Initialize Kinematic Bicycle filter
        v_filter_kin_.set_problem_size(n_, m_kin_, p_);
        v_filter_kin_.set_initial_data(x_initial_, P_initial_, u_initial_kin_);

        B_kin_(0,0) = 1;
        B_kin_(1,1) = lr_ / L_;
        v_filter_kin_.set_process_matrices(M_kin_, B_kin_, Q_kin_);
        v_filter_kin_.set_measurement_matrices(H_, R_kin_);

        // Initialize Dynamic Bicycle filter
        v_filter_dyn_.set_problem_size(n_, m_dyn_, p_);
        v_filter_dyn_.set_initial_data(x_initial_, P_initial_, u_initial_dyn_);

        B_dyn_(0,0) = 1;
        B_dyn_(1,1) = 1;
        B_dyn_(1,2) = 1;
        v_filter_dyn_.set_process_matrices(M_dyn_, B_dyn_, Q_dyn_);
        v_filter_dyn_.set_measurement_matrices(H_, R_dyn_);
    };

    /**
     * @brief Speed estimation
     * Estimates car's longitudinal and lateral velocities combining kinematic and dynamic
     * bicycle models based on car's current speed.
     */
    Vector2d estimate_speed(double ax, double r, double delta, double delta_der, 
        double wFL, double wFR, double wRL, double wRR, double inv_speed, bool kSimulation){
        // Kinematic bicycle
        VectorXd u_kin(m_kin_), z_kin(p_), x_kin(n_);
        MatrixXd P_kin(n_,n_);
        
        M_kin_(1,0) = lr_ / L_ * delta_der;
        v_filter_kin_.update_model_matrix(M_kin_);

        u_kin << ax, ax*delta;

        if(!kSimulation && std::abs(vx_) < 5) {
            z_kin << inv_speed, lr_ / L_ * std::tan(delta) * vx_; 
        } else {
            z_kin << (wFL + wFR + wRL + wRR)/4, lr_ / L_ * std::tan(delta) * vx_;
        }

        v_filter_kin_.estimate_state(u_kin, z_kin);
        x_kin = v_filter_kin_.get_estimated_state();
        P_kin = v_filter_kin_.get_estimated_covariance();

        // Dynamic bicycle
        VectorXd u_dyn(m_dyn_), z_dyn(p_), x_dyn(n_);
        MatrixXd P_dyn(n_,n_);

        M_dyn_(0,1) = r;
        M_dyn_(1,0) = -r;
        v_filter_dyn_.update_model_matrix(M_dyn_);

        B_dyn_(0,1) = -std::sin(delta);
        B_dyn_(1,1) = std::cos(delta);
        v_filter_dyn_.update_control_matrix(B_dyn_);

        double a_f = std::atan2(vy_ + lf_ * r, vx_) - delta;
        double a_r = std::atan2(vy_ - lr_ * r, vx_) - delta;
        double F_fy = D_lat_ * std::sin(C_lat_ * std::atan(B_lat_ * a_f));
        double F_ry = D_lat_ * std::sin(C_lat_ * std::atan(B_lat_ * a_r));
        u_dyn << ax, F_fy / m_, F_ry / m_;

        if(!kSimulation && std::abs(vx_) < 5) {     
            z_dyn << inv_speed, 
                     (vx_ * (lf_*cf_ + lr_*cr_)) / (m_*vx_*vx_ + cf_*lf_*lf_ + cr_*lr_*lr_) * delta; 
        } else {
            z_dyn << (wFL + wFR + wRL + wRR)/4, 
                     (vx_ * (lf_*cf_ + lr_*cr_)) / (m_*vx_*vx_ + cf_*lf_*lf_ + cr_*lr_*lr_) * delta;
        }

        v_filter_dyn_.estimate_state(u_dyn, z_dyn);
        x_dyn = v_filter_dyn_.get_estimated_state();
        P_dyn = v_filter_dyn_.get_estimated_covariance();

        // Estimated state
        double lambda = std::clamp((vx_ - vx_blend_min_) / (vx_blend_max_ - vx_blend_min_), 0., 1.);
        VectorXd x_est = (1 - lambda) * x_kin + lambda * x_dyn;
        vx_ = x_est(0);
        vy_ = x_est(1);

        // Update estimated state and covariance
        v_filter_kin_.update_state(x_est);
        v_filter_dyn_.update_state(x_est);

        MatrixXd P_est = (1 - lambda) * P_kin + lambda * P_dyn;
        v_filter_kin_.update_covariance(P_est);
        v_filter_dyn_.update_covariance(P_est);     

        return x_est;
    }
};

