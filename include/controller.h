/**
 *	\file include/controller.h
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#pragma once

#include <string>                 // #include directives    

#include "parameters.h"           // #include parameters

#include <Eigen/Dense>            // #include module
#include <Eigen/SparseCore>
#include <qpOASES.hpp>

using namespace Eigen;            // using namespace
using namespace std;

class controller
{
    private:
        // Private class members
        int Np, p, n, m;
        double Q, d;
        MatrixXd Ab;
        MatrixXd Xub;
        MatrixXd Ulb, Uub;
        VectorXd ulin;
        MatrixXd M1, M2;
        SparseMatrix<double> C;
        qpOASES::QProblem Qp; 
        qpOASES::real_t H_qp[100*100];
        
        // Private class method
        void initialize(MatrixXd &A, MatrixXd &B, MatrixXd &qlin, double R, double QN);
    public:
        // Constructors
        controller(MatrixXd &A, MatrixXd &B, SparseMatrix<double> &Cpar, double d_par, double Q_par, double R, double QN, Matrix<double, P::n_states_lift * P::n_prediction, 1> &Xub_par,
            Matrix<double, P::n_control_input * P::n_prediction, 1> &Ulb_par, Matrix<double, P::n_control_input * P::n_prediction, 1> &Uub_par,
            Matrix<double, P::n_control_input * P::n_prediction, 1> &ulin_par, Matrix<double, P::n_output * P::n_prediction , 1> &qlin);
        
        controller(MatrixXd &A, MatrixXd &B, SparseMatrix<double> &Cpar, double d_par, double Q_par, double R, double QN, int Npred, Matrix<double, P::n_states_lift, 1> &Xub_par,
            Matrix<double, P::n_control_input, 1> &Ulb_par, Matrix<double, P::n_control_input, 1> &Uub_par,
            Matrix<double, P::n_control_input, 1> &ulin_par, Matrix<double, P::n_output, 1> &qlin);
        
        controller(MatrixXd &A, MatrixXd &B, SparseMatrix<double> &Cpar, double d_par, double Q_par, double R, double QN, int Npred, double Xub_par,
            double Ulb_par, double Uub_par, double ulin_par, double qlin_par);
        
        // Public class method
        double getControlInput(VectorXd &x0, double yrr);
};
