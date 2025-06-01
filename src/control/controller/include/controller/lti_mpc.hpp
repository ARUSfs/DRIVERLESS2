#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "controller/utils.hpp"

class LtiMpc
{
public:
    LtiMpc(){
        C.resize(2,6);
        C << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;    
    }
    
    /**
     * @brief Calculate steering angle command using LTI-MPC
     */
    double calculate_control(const double &delta, double &delta_v, double &vy, double &r){

        delta_ = delta;
        delta_v_ = delta_v;

        x_0_.resize(6,1);
        x_0_ << 0, vy, 0, r, delta_, delta_v_;

        // Linearize and discretize model

        A = Eigen::MatrixXd::Zero(6,6);
        B = Eigen::MatrixXd::Zero(6,1);

        linearize_model(prediction_speed_(0), A, B);

        discretize_model(A, B, kTsMPC, Ad, Bd);

        // Define cost matrices

        Q.resize(C.rows(),C.rows());
        Q << kCostLateralDeviation, 0, 0, kCostAngularDeviation;
        R = kCostSteeringDelta * Eigen::MatrixXd::Identity(kPredictionHorizon, kPredictionHorizon);

        // MPC matrices construction 

        Eigen::MatrixXd PSI = Eigen::MatrixXd::Zero(C.rows() * kPredictionHorizon, Ad.cols());
        Eigen::MatrixXd YPS = Eigen::MatrixXd::Zero(C.rows() * kPredictionHorizon, Bd.cols());
        Eigen::MatrixXd THETA = Eigen::MatrixXd::Zero(C.rows() * kPredictionHorizon, Bd.cols() * kPredictionHorizon);
        Eigen::MatrixXd Q_THETA = Eigen::MatrixXd::Zero(Q.rows() * kPredictionHorizon, Bd.cols() * kPredictionHorizon);
        Eigen::MatrixXd Q_YPS = Eigen::MatrixXd::Zero(Q.rows() * kPredictionHorizon, Bd.cols());

        Eigen::MatrixXd Apow = Eigen::MatrixXd::Identity(Ad.rows(),Ad.cols());

        YPS.topRows(C.rows()) = C * Bd;

        for (int i = 0; i < kPredictionHorizon; i++){
            if (i>0){YPS.middleRows(i * C.rows(), C.rows()) = YPS.middleRows((i-1)*C.rows(), C.rows()) + C * Apow * Bd;}

            Apow *= Ad;

            linearize_model(prediction_speed_(i+1), A, B);

            discretize_model(A, B, kTsMPC, Ad, Bd);  

            PSI.middleRows(i * C.rows(), C.rows()) = C * Apow;
            Q_YPS.middleRows(i * Q.rows(), Q.rows()) = Q * YPS.middleRows(i * C.rows(), C.rows());            
        }

        for (int i = 0; i < kPredictionHorizon; i++){
            THETA.block(i * C.rows(), i * Bd.cols(), (kPredictionHorizon - i) * C.rows(), Bd.cols()) = YPS.topRows((kPredictionHorizon - i) * C.rows());
            Q_THETA.block(i * Q.rows(), i * Bd.cols(), (kPredictionHorizon - i) * Q.rows(), Bd.cols()) = Q_YPS.topRows((kPredictionHorizon - i) * Q.rows());
        }

        // Solve least squares problem

        Eigen::MatrixXd A_mpc = 2 * (Q_THETA.transpose() * THETA + R);
        Eigen::MatrixXd b_mpc = 2 * Q_THETA.transpose() * (target_trajectory_ - PSI * x_0_ - YPS * delta_);

        U = A_mpc.llt().solve(b_mpc);

        double delta_target = delta_ + U(0,0);

        for (int i = 1; i < kCompensationSteps + 1; i++){
            delta_target += U(i,0);
        }

        return delta_target;
    }

    /**
     * @brief Create discrete local reference trajectory
     */
    void set_reference_trajectory(const std::vector<Point> &global_reference_trajectory, const std::vector<float> &vx_profile, const std::vector<float> &s, 
        const Point &position, double &yaw, double &vx, size_t index_global){

        if (vx >= 2.0){
            v_linearisation = vx;
        } else {
            v_linearisation = 2.0;
        }
        prediction_speed_ = Eigen::VectorXd::Zero(kPredictionHorizon+1);
        prediction_speed_(0) = v_linearisation;

        double s_predicted = s[index_global];
        // double ds = v_linearisation * kTsMPC;

        target_trajectory_.resize(2*kPredictionHorizon);

        for (int i = 0; i < kPredictionHorizon; i++){
           
            Point xy_interp = interpolate_data(global_reference_trajectory, vx_profile, s, s_predicted);

            target_trajectory_(2*i) = - (xy_interp.x - position.x) * std::sin(yaw) + (xy_interp.y - position.y) * std::cos(yaw);
            target_trajectory_(2*i+1) = (std::abs(yaw_interp - yaw)<1) ? yaw_interp - yaw : 0.0;

            prediction_speed_(i+1) = v_interp;

            s_predicted += v_interp * kTsMPC;
            
            if (s_predicted >= s[s.size()-1]) {s_predicted -= s[s.size()-1];}
        }
    }

    /**
     * @brief Set configurable parameters from config file
     */
    void set_params(double cost_lateral, double cost_angular, double cost_delta, int compensation_steps,
        int prediction_horizon, double ts_mpc, double cornering_stiffness_front, double cornering_stiffness_rear,
        double wheelbase, double r_cdg, double mass, double Izz, double steer_u, double steer_delta, double steer_delta_v){

        kCostAngularDeviation = cost_angular;
        kCostLateralDeviation = cost_lateral;
        kCostSteeringDelta = cost_delta;
        kCompensationSteps = compensation_steps;
        kPredictionHorizon = prediction_horizon;
        kTsMPC = ts_mpc;
        kMass = mass;
        kWheelbase = wheelbase;
        kIzz = Izz;
        kWeightDistributionRear = r_cdg;
        kCorneringStiffnessF = cornering_stiffness_front;
        kCorneringStiffnessR = cornering_stiffness_rear;
        kSteerModelU = steer_u;
        kSteerModelDelta = steer_delta;
        kSteerModelDeltaV = steer_delta_v;

        kLf = kWheelbase * kWeightDistributionRear;
        kLr = kWheelbase - kLf;
    }

private:
    // Variables
    double v_linearisation;
    double delta_{0.0};
    double delta_v_{0.0};
    double yaw_interp{0.0};
    double v_interp{0.0};

    Eigen::VectorXd prediction_speed_;

    Eigen::VectorXd x_0_;
    Eigen::VectorXd target_trajectory_;

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;

    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;

    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd U;

    // Parameters
    double kCostLateralDeviation;
    double kCostAngularDeviation;
    double kCostSteeringDelta;
    int kCompensationSteps;
    int kPredictionHorizon;
    double kTsMPC;

    double kCorneringStiffnessF;
    double kCorneringStiffnessR;

    double kWheelbase;
    double kWeightDistributionRear;
    double kLf;
    double kLr;

    double kMass;
    double kIzz;

    double kSteerModelU;
    double kSteerModelDelta;
    double kSteerModelDeltaV;

    /**
     * @brief Linearize vehicle model using current speed
     */
    void linearize_model(double v_linearisation, Eigen::MatrixXd &Ac, Eigen::MatrixXd &Bc){

        Ac.resize(6, 6);
        Bc.resize(6, 1);

        Ac << 0, 1, v_linearisation, 0, 0, 0,
        0, (kCorneringStiffnessF+kCorneringStiffnessR)/(kMass*v_linearisation), 0, 
        (kLf*kCorneringStiffnessF - kLr*kCorneringStiffnessR)/(kMass*v_linearisation) - v_linearisation, -kCorneringStiffnessF/kMass, 0, 
        0, 0, 0, 1, 0, 0,
        0, (kLf*kCorneringStiffnessF - kLr*kCorneringStiffnessR)/(kIzz*v_linearisation), 0, 
        (std::pow(kLf,2)*kCorneringStiffnessF + std::pow(kLr,2)*kCorneringStiffnessR)/(kIzz*v_linearisation), -kLf*kCorneringStiffnessF/kIzz, 0, 
        0, 0, 0, 0, 0, 1,
        0, 0, 0, 0, kSteerModelDelta, kSteerModelDeltaV;

        Bc << 0, 0, 0, 0, 0, kSteerModelU;
        
    }

    /**
     * @brief Discretize linear model using matrix exponential
     */
    void discretize_model(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const double& dT, Eigen::MatrixXd& Phi, Eigen::MatrixXd& Gamma) {

        int m = A.rows();
        int n = A.cols();
        int nb = B.cols();

        Eigen::MatrixXd tmp3(m + nb, n + nb);
        tmp3.setZero();
        tmp3.topLeftCorner(n, n) = A * dT;
        tmp3.topRightCorner(n, nb) = B * dT;

        Eigen::MatrixXd s = tmp3.exp().eval();  // Eigen's built-in Padé approximation

        Phi = s.topLeftCorner(n, n);
        Gamma = s.topRightCorner(n, nb);

    }

    /**
     * @brief Interpolate variables to match with predicted position
     */
    Point interpolate_data(const std::vector<Point> &XYdata, const std::vector<float> &vx_profile, const std::vector<float> &s, const double &s0) {

        if (s0 <= s.front()) return XYdata.front();
        if (s0 >= s.back()) return XYdata.back();

        Point point;

        for (size_t i = 0; i < s.size() - 1; i++) {
            if (s0 >= s[i] && s0 <= s[i + 1]) {
                double alpha = (s0 - s[i]) / (s[i + 1] - s[i]);
                double x_interp = XYdata[i].x + alpha * (XYdata[i + 1].x - XYdata[i].x);
                double y_interp = XYdata[i].y + alpha * (XYdata[i + 1].y - XYdata[i].y);

                yaw_interp = std::atan2((XYdata[i+1].y-XYdata[i].y),(XYdata[i+1].x-XYdata[i].x));

                v_interp = std::max(2.0, vx_profile[i] + alpha * (vx_profile[i+1] - vx_profile[i]));

                point.x = x_interp;
                point.y = y_interp;
            }
        }
        return point;
    }
};