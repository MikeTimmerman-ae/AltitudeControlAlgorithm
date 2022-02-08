/**
 *	\file src/controller.cpp
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#include "../include/parameters.h"  // #include header
#include "../include/controller.h"  // #include header

#include <iostream>                 // #include directives
#include <vector>
#include <math.h>
#include <string>

#include <qpOASES.hpp>                       // #include modules
#include <Eigen/Dense>
#include <Eigen/SparseCore>

using namespace std;
using namespace Eigen;


controller::controller(MatrixXd &A, MatrixXd &B, SparseMatrix<double> &C, double d_par, double Q_par, double R, double QN, Matrix<double, P::n_states, P::n_prediction> &Xub_par,
    Matrix<double, P::n_control_input, P::n_prediction> &Ulb_par, Matrix<double, P::n_control_input, P::n_prediction> &Uub_par,
    Matrix<double, P::n_control_input * P::n_prediction , 1> &ulin_par, Matrix<double, P::n_output * P::n_prediction , 1> &qlin)
{
    Xub = Xub_par;                      // Save state bounds as controller parameters
    Ulb = Ulb_par; Uub = Uub_par;       // Save control bounds as controller paramters
    d = d_par; Q = Q_par;               // Save wieght Q and affine term in the dynamics d as parameters
    ulin = ulin_par;                    // Save linear term in the cost as controller parameter
    
    Np = P::n_prediction;               // Number of Control Horizon points
    p = P::n_output;                    // Number of outputs
    const int n = P::n_states_lift;     // Number of states
    const int m = P::n_control_input;   // Number of control inputs

    VectorXd x0(n);                     // Dummy Variable

}

controller::controller(MatrixXd &A, MatrixXd &B, SparseMatrix<double> &C, double d_par, double Q_par, double R, double QN, int Npred, Matrix<double, P::n_states, 1> &Xub_par,
    Matrix<double, P::n_control_input, 1> &Ulb_par, Matrix<double, P::n_control_input, 1> &Uub_par,
    Matrix<double, P::n_control_input, 1> &ulin_par, Matrix<double, P::n_output, 1> &qlin_par) 
{    
    Np = Npred;                         // Number of Control Horizon points
    p = P::n_output;                    // Number of outputs
    const int n = P::n_states_lift;     // Number of states
    const int m = P::n_control_input;   // Number of control inputs
    d = d_par; Q = Q_par;               // Save wieght Q and affine term in the dynamics d as parameters
    
    VectorXd x0(n);                     // Dummy Variable

    // Handle state boundary matrices; convert matrices from n x 1 --> n x Np
    Xub = Xub_par.replicate(1, Np);

    // Handle control input boundary matrices; convert matrices from m x 1 --> m x Np
    Ulb = Ulb_par.replicate(1, Np);
    Uub = Uub_par.replicate(1, Np);

    // Linear term in the cost; convert matrix from m x 1 --> m*Np x 1
    ulin = ulin_par.replicate(Np, 1);

    // Quadratic term in cost; convert matrix from p x 1 --> p*Np x 1
    VectorXd qlin = qlin_par.replicate(Np, 1);

}

controller::controller(MatrixXd &A, MatrixXd &B, SparseMatrix<double> &C, double d_par, double Q_par, double R, double QN, int Npred,
            double Ulb_par, double Uub_par, double Xub_par, double ulin_par, double qlin_par)
{        
    Np = Npred;                         // Number of Control Horizon points
    p = P::n_output;                    // Number of outputs
    const int n = P::n_states_lift;     // Number of states
    const int m = P::n_control_input;   // Number of control inputs
    d = d_par; Q = Q_par;               // Save wieght Q and affine term in the dynamics d as parameters
    
    VectorXd x0(n);                     // Dummy Variable

    // Handle state boundary matrices; convert matrices from 1 x 1 --> n x Np
    Xub = MatrixXd::Constant(n, Np, Xub_par);

    // Handle control input boundary matrices; convert matrices from 1 x 1 --> m x Np
    Ulb = MatrixXd::Constant(m, Np, Ulb_par);
    Uub = MatrixXd::Constant(m, Np, Uub_par);

    // Linear term in the cost; convert matrix from m x 1 --> m*Np x 1
    ulin = VectorXd::Constant(m*Np, ulin_par);

    // Quadratic term in cost; convert matrix from m x 1 --> p*Np x 1
    VectorXd qlin = VectorXd::Constant(p*Np, qlin_par);


}
