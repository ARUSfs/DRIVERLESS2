#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "Point.h"
#include <unsupported/Eigen/MatrixFunctions>

class LtiMpc
{
public:
    LtiMpc(){
        C.resize(2,6);
        C << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;    
    }

    double calculate_control(const double &delta, double &delta_v, double &vy, double &r){

        delta_ = delta;
        delta_v_ = delta_v;

        x_0_.resize(6,1);

        x_0_ << 0, vy, 0, r, delta_, delta_v_;

        A = Eigen::MatrixXd::Zero(6,6);
        B = Eigen::MatrixXd::Zero(6,1);

        linearize_model(v_linearisation, A, B);

        discretize_model(A, B, kTsMPC, Ad, Bd);

        Q.resize(C.rows(),C.rows());
        Q << kCostLateralDeviation, 0, 0, kCostAngularDeviation;
        R = kCostSteeringDelta * Eigen::MatrixXd::Identity(kPredictionHorizon, kPredictionHorizon);

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

        double delta_target = delta_ + U(0,0) + U(1,0);

        return delta_target;
    }

    void set_reference_trajectory(const std::vector<Point> &new_path, const std::vector<float> &s, 
        const Point &position, double &yaw, double &vx, size_t index_global){

        global_reference_trajectory_.clear();

        s_.clear();
        global_reference_trajectory_ = new_path;
        s_ = s;
        double s_predicted = s_[index_global];

        if (vx >= 2.0){
            v_linearisation = vx;
        } else {
            v_linearisation = 2.0;
        }

        double ds = v_linearisation * kTsMPC;

        target_trajectory_.resize(2*kPredictionHorizon);

        for (int i = 0; i < kPredictionHorizon; i++){
           
            Point xy_interp = interpolate_data(global_reference_trajectory_, s_, s_predicted);

            target_trajectory_(2*i) = - (xy_interp.x - position.x) * std::sin(yaw) + (xy_interp.y - position.y) * std::cos(yaw);
            target_trajectory_(2*i+1) = (std::abs(yaw_interp - yaw)<1) ? yaw_interp - yaw : 0.0;

            s_predicted += ds;
            if (s_predicted >= s_[s_.size()-1]) {s_predicted -= s_[s_.size()-1];}
        }
    }

    void set_params(double cost_lateral, double cost_angular, double cost_delta){
        kCostAngularDeviation = cost_angular;
        kCostLateralDeviation = cost_lateral;
        kCostSteeringDelta = cost_delta;
    }

private:
    int kPredictionHorizon = 65;
    double kTsMPC = 0.02;

    std::vector<Point> global_reference_trajectory_;
    std::vector<float> s_;

    double kCorneringStiffnessF = -24276;
    double kCorneringStiffnessR = -20332;

    double kWheelbase = 1.533;
    double kLf = kWheelbase*0.5073;
    double kLr = kWheelbase-kLf;

    double kMass = 270;
    double kIzz = 180;

    double v_linearisation;
    double delta_{0.0};
    double delta_v_{0.0};

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

    // Configurable parameters
    double kCostLateralDeviation;
    double kCostAngularDeviation;
    double kCostSteeringDelta;

    double yaw_interp{0.0};

    void linearize_model(double v_linearisation, Eigen::MatrixXd &Ac, Eigen::MatrixXd &Bc){

        Ac.resize(6, 6);
        Bc.resize(6, 1);

        Ac << 0, 1, v_linearisation, 0, 0, 0,
        0, (kCorneringStiffnessF+kCorneringStiffnessR)/(kMass*v_linearisation),0, (kLf*kCorneringStiffnessF - kLr*kCorneringStiffnessR)/(kMass*v_linearisation) - v_linearisation, -kCorneringStiffnessF/kMass, 0,
        0, 0, 0, 1, 0, 0,
        0, (kLf*kCorneringStiffnessF - kLr*kCorneringStiffnessR)/(kIzz*v_linearisation), 0, (std::pow(kLf,2)*kCorneringStiffnessF + std::pow(kLr,2)*kCorneringStiffnessR)/(kIzz*v_linearisation),
        -kLf*kCorneringStiffnessF/kIzz, 0, 0, 0, 0, 0,
        0, 1,
        0, 0, 0, 0, -318.5, -26.24;

        Bc << 0, 0, 0, 0, 0, 319.2;
        
    }

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

    Point interpolate_data(const std::vector<Point> &XYdata, const std::vector<float> &s, const double &s0) {
        if (s.empty() || XYdata.empty() || s.size() != XYdata.size()) {
            throw std::invalid_argument("Vectors s and XYdata must have the same non-zero size.");
        }

        if (s0 <= s.front()) return XYdata.front();
        if (s0 >= s.back()) return XYdata.back();

        Point point;

        for (size_t i = 0; i < s.size() - 1; i++) {
            if (s0 >= s[i] && s0 <= s[i + 1]) {
                double alpha = (s0 - s[i]) / (s[i + 1] - s[i]);
                double x_interp = XYdata[i].x + alpha * (XYdata[i + 1].x - XYdata[i].x);
                double y_interp = XYdata[i].y + alpha * (XYdata[i + 1].y - XYdata[i].y);

                yaw_interp = std::atan2((XYdata[i+1].y-XYdata[i].y),(XYdata[i+1].x-XYdata[i].x));

                point.x = x_interp;
                point.y = y_interp;
            }
        }
        return point;
    }
};