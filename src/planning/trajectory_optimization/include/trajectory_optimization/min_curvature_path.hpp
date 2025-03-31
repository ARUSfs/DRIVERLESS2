/**
 * @file min_curvature_path.hpp
 * @author José Manuel Landero Plaza (josemlandero05@gmail.com)
 * @brief Namespace for implementation of the optimized path generator and all the necessary auxiliar methods
 * @date 6-11-2024
 */
#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "libInterpolate/Interpolate.hpp"
#include "qpmad/solver.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
using namespace std;
using namespace Eigen;

namespace MinCurvaturepath {
    
    //Declaration of auxiliar methods

    /**
     * @brief Smooths the given path increasing the number of points and 
     * returns the smooth path in terms of centerline and path boundaries
     * 
     * @param  x X coordinates of the given path points
     * @param  y Y coordinates of the given path points
     * @param  twr Track width allowed to the right of each point
     * @param  twl Track width allowed to the left of each point
     * 
     * @return MatrixXd Matrix containing track data: [xt, yt, xin, yin, xout, yout]
     */
    MatrixXd process_track_data(VectorXd x, VectorXd y, VectorXd twr, VectorXd twl, int n_seg);

    /**
     * @brief Calculates the difference between each pair of consecutive elements of each column
     * 
     * @return MatrixXd 
     */
    MatrixXd diff_col(MatrixXd E);

    /**
     * @brief Calculates the cumulative sum of a vector
     * 
     * @return VectorXd 
     */
    VectorXd cumsum(VectorXd v);

    /**
     * @brief Calculates one-dimensional numerical gradient of vector f (∂f/∂x)
     * 
     * @return VectorXd 
     */
    VectorXd gradient(VectorXd f);

    /**
     * @brief Generates the H matrix of our quadratic problem
     * 
     * @return MatrixXd
     */
    MatrixXd matrixH(VectorXd delx, VectorXd dely);

    /**
     * @brief Generates the B matrix of our quadratic problem
     * 
     * @return MatrixXd
     */
    VectorXd vectorB(VectorXd xin, VectorXd yin, VectorXd delx, VectorXd dely);

    /**
     * @brief Solves the quadratic optimization problem using an external solver
     * 
     * @return VectorXd Parameter vector (aplha) that determines the resulting trajectory points 
     * (traj_x = xin + aplha*delx, traj_y = yin + alpha*dely)
     */
    VectorXd qpmad_solver(MatrixXd H, VectorXd B);

    /**
     * @brief Main function that calculates the minimal curvature path 
     * using several auxiliar methods
     * 
     * @param  x X coordinates of the given path points
     * @param  y Y coordinates of the given path points
     * @param  twr Track width allowed to the right of each point
     * @param  twl Track width allowed to the left of each point
     * 
     * @return MatrixXd Matrix containing optimal path: [traj_x, traj_y]
     */
    MatrixXd get_min_curvature_path(VectorXd x, VectorXd y, VectorXd twr, VectorXd twl, int n_seg){          
        //First, we process track data 
        MatrixXd track_data = process_track_data(x, y, twr, twl, n_seg);

        //Then, we form the matrices which will define the quadratic optimization problem
        VectorXd xin = track_data.col(2);
        VectorXd yin = track_data.col(3);
        VectorXd xout = track_data.col(4);
        VectorXd yout = track_data.col(5);
        
        VectorXd delx = xout - xin;
        VectorXd dely = yout - yin;

        MatrixXd H = matrixH(delx, dely);
        VectorXd  B = vectorB(xin, yin, delx, dely);

        //Solve the quadratic problem
        VectorXd resMCP = qpmad_solver(H,B);
        int m = resMCP.size();

        //Coordinates for the resultant curve
        VectorXd xresMCP = VectorXd::Zero(m);   
        VectorXd yresMCP = VectorXd::Zero(m);

        for(int i = 0; i < m; i++){
            xresMCP(i) = xin(i) + resMCP(i)*delx(i);
            yresMCP(i) = yin(i) + resMCP(i)*dely(i);
        }

        // MatrixXd res(m,2);
        // res << xresMCP, yresMCP;

        // PARA DEBUGGEAR
        MatrixXd res(m,6);
        res << xresMCP, yresMCP, xin, yin, xout, yout;

        return res;       
    }

    //Implementation of auxiliar methods
    MatrixXd process_track_data(VectorXd x, VectorXd y, VectorXd twr, VectorXd twl, int n_seg){
        //Interpolate data to get finer curve with equal distances between each segment
        //Higher number of segments causes trajectory to follow the reference line
        int n = x.size();

        MatrixXd path_x_y(n,2); path_x_y << x, y;

        MatrixXd difs2 = diff_col(path_x_y).array().square().matrix();
        VectorXd step_lengths = (difs2.col(0)+difs2.col(1)).cwiseSqrt();

        VectorXd temp_step = step_lengths;
        step_lengths.conservativeResize(step_lengths.size()+1);
        step_lengths << 0, temp_step;   // add the starting point

        VectorXd cumulative_len = cumsum(step_lengths); 
        VectorXd final_step_locks = VectorXd::LinSpaced(n_seg, 0, cumulative_len(cumulative_len.size()-1));
    
        int m = final_step_locks.size();
        
        MatrixXd final_path_x_y(m, 2);
        _1D::LinearInterpolator<double> interp;
        interp.setData(cumulative_len, x);
        for(int i = 0; i < m; i++){
            final_path_x_y(i, 0) = interp(final_step_locks(i));
        }

        interp.setData(cumulative_len, y);
        for(int i = 0; i < m; i++){
            final_path_x_y(i, 1) = interp(final_step_locks(i));
        }

        VectorXd xt = final_path_x_y.col(0);
        VectorXd yt = final_path_x_y.col(1);

        
        _1D::CubicSplineInterpolator<double> interp_c;
        interp_c.setData(cumulative_len, twr);
        VectorXd twrt(m);
        for(int i = 0; i < m; i++){
            twrt(i) = interp_c(final_step_locks(i));
        }

        interp_c.setData(cumulative_len, twl);
        VectorXd twlt(m);
        for(int i = 0; i < m; i++){
            twlt(i) = interp_c(final_step_locks(i));
        }

        //normal direction for each vertex
        VectorXd dx = gradient(xt); 
        auto dx_a = dx.array();
        VectorXd dy = gradient(yt); 
        auto dy_a = dy.array();
        VectorXd dL = (dx_a*dx_a + dy_a*dy_a).sqrt().matrix();

        //Offset data
        MatrixXd offset(m, 2);
        offset << -twrt, twlt;
        VectorXd xin = VectorXd::Zero(m); 
        VectorXd yin = VectorXd::Zero(m);
        VectorXd xout = VectorXd::Zero(m);
        VectorXd yout = VectorXd::Zero(m);

        VectorXd aux_x = (dy.array()/dL.array()).matrix();
        VectorXd aux_y = (dx.array()/dL.array()).matrix();

        for(int i = 0; i < m; i++){
            VectorXd xinv, yinv, xoutv, youtv;
            
            xinv = -offset(i,0)*aux_x + xt;    //get inner offset curve
            yinv = offset(i,0)*aux_y + yt;

            xoutv = -offset(i,1)*aux_x + xt;   //get outer offset curve
            youtv = offset(i,1)*aux_y + yt;

            xin(i) = xinv(i);
            yin(i) = yinv(i);
            xout(i) = xoutv(i);
            yout(i) = youtv(i);
        }
        
        MatrixXd track_data(m,6);
        track_data << xt, yt, xin, yin, xout, yout;

        return track_data;
    }

    MatrixXd diff_col(MatrixXd E) { 
        MatrixXd E1 = E.block(0, 0, E.rows()-1, E.cols());
        MatrixXd E2 = E.block(1, 0, E.rows()-1, E.cols());
        return E2 - E1;
    }

    VectorXd cumsum(VectorXd v){
        int n = v.size(); VectorXd res(n);

        res(0) = v(0);
        for(int i = 1; i < n; i++){
            res(i) = v.head(i+1).sum();
        }

        return res;
    }

    VectorXd gradient(VectorXd f){
        int n = f.size();
        VectorXd grad(n);

        grad(0) = f(1)-f(0);
        for(int i = 1; i < n-1; i++){
            grad(i) = (f(i+1)-f(i-1))/2;
        }
        grad(n-1) = f(n-1)-f(n-2);

        return grad;
    }

    MatrixXd matrixH(VectorXd delx, VectorXd dely){
        int n = delx.size();
        MatrixXd H = MatrixXd::Zero(n,n);

        for(int i = 1; i < n-1; i++ ){
            //First row
            H(i-1,i-1) = H(i-1,i-1) + pow(delx(i-1),2)     + pow(dely(i-1),2);
            H(i-1,i)   = H(i-1,i)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i);
            H(i-1,i+1) = H(i-1,i+1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1);

            //Second row
            H(i,i-1)   = H(i,i-1)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i);
            H(i,i)     = H(i,i )    + 4*pow(delx(i),2)    + 4*pow(dely(i),2);
            H(i,i+1)   = H(i,i+1)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1);

            //Third row
            H(i+1,i-1) = H(i+1,i-1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1);
            H(i+1,i)   = H(i+1,i)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1);
            H(i+1,i+1) = H(i+1,i+1) + pow(delx(i+1),2)    + pow(dely(i+1),2);
        }

        return H;
    }

    VectorXd vectorB(VectorXd xin, VectorXd yin, VectorXd delx, VectorXd dely){
        int n = delx.size();
        VectorXd B = VectorXd::Zero(n);

        for(int i = 1; i < n-1; i++){
            B(i-1) = B(i-1) + 2*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i-1) + 2*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i-1);
            B(i)   = B(i)   - 4*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i)   - 4*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i);
            B(i+1) = B(i+1) + 2*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i+1) + 2*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i+1);
        }

        return B;
    }

    VectorXd qpmad_solver(MatrixXd H, VectorXd B){
        //Define constraints
        int n = H.rows();
        VectorXd lb = VectorXd::Zero(n);
        VectorXd ub = VectorXd::Ones(n);

        //If start and end points are the same. 
        //Solver doesn't handle equality constraints, but we can implement it as two inequalities
        // beq <= Aeq*res <= beq
        MatrixXd Aeq = MatrixXd::Zero(4,n);
        Aeq(0,0) = 1;
        Aeq(1,1) = 1;
        Aeq(2,n-2) = 1;
        Aeq(3,n-1) = 1;
        VectorXd beq(4);
        beq << 0.5, 0.5, 0.5, 0.5;

        VectorXd Alb(4), Aub(4);
        Alb << beq;
        Aub << beq;

        //Solver
        VectorXd res;
        qpmad::Solver solver;

        H << 2*H;
        qpmad::Solver::ReturnStatus status = solver.solve(res, H, B, lb, ub, Aeq, Alb, Aub);
        if (status != qpmad::Solver::OK)
        {
            cerr << "Error" << endl;
        }

        return res;
    }    
};

