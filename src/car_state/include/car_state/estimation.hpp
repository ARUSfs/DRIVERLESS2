#include <Eigen/Dense>
using namespace Eigen;

class Estimation
{
    private:
        //Private atributes to estimate next state:

        //Sensor data
        double v_front_right_ = 0;
        double v_front_left_ = 0;
        double v_rear_right_ = 0;
        double v_rear_left_ = 0;
        double ax_ = 0;
        double ay_ = 0;

        //Previous state
        Vector2d x_prev_;
        Matrix2d P_prev_;

        //Additional parameters
        Matrix2d Q_;
        Matrix2d R_;
        double dt_;

    public:
        Estimation()
        {   
            x_prev_ = Vector2d::Zero();
            P_prev_ = Matrix2d::Zero(); // ?????????????????????????????????
            Q_ = 0.1*Matrix2d::Identity();
            R_ = 0.5*Matrix2d::Identity();
            dt_ = 0.1; // ?????????????????????????????

        }

        void set_data(double vfr, double vfl, double vrr, double vrl, double ax, double ay)
        {
            v_front_right_ = 0;
            v_front_left_ = 0;
            v_rear_right_ = 0;
            v_rear_left_ = 0;
            ax_ = 0;
            ay_ = 0;
        }

        Vector2d kalman_velocity_estimation()
        {
            
            return x_prev_;
        }
        
};