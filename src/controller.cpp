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
    Np = P::n_prediction;               // Number of Control Horizon points
    p = P::n_output;                    // Number of outputs

    Xub = Xub_par;                      // Save state bounds as controller parameters
    Ulb = Ulb_par; Uub = Uub_par;       // Save control bounds as controller paramters
    d = d_par; Q = Q_par;               // Save wieght Q and affine term in the dynamics d as parameters
    ulin = ulin_par;                    // Save linear term in the cost as controller parameter

    
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

void controller::initialize(MatrixXd &A, MatrixXd &B, MatrixXd &qlin, double R, double QN) {
    /* Initialize Controller*/

    const int n = P::n_states_lift;     // Number of states
    const int m = P::n_control_input;   // Number of control inputs

    VectorXd x0(n);                     // Dummy Variable

    // Create MPC matrices Ab and Bb
    Ab = MatrixXd(Np*n, n); Ab(seq(0, n-1), seq(0, n-1)) = A;
    MatrixXd Bb(Np*n, Np*m); Bb(seq(0, n-1), seq(0, m-1)) = B;
    for (int i = 1; i < Np; ++i) {
        Ab(seq(i*n, (i+1)*n-1), seq(0,n-1)) = Ab(seq((i-1)*n, i*n-1), seq(0,n-1)) * A;
        Bb(seq(i*n, (i+1)*n-1), seq(0,Np*m-1)) = A * Bb(seq((i-1)*n, i*n-1), seq(0,Np*m-1));
        Bb(seq(i*n, (i+1)*n-1), seq(i*m, (i+1)*m-1)) = B;
    }      
    
    // Build the controller (Matrices Qb, Cb, Rb --> M1 and M2)
    SparseMatrix<double> Qb(p*Np,p*Np);
    for (int i = 0; i < Np*p; ++i) {Qb.insert(i, i) = Q;}
    Qb.insert(p*Np-1, p*Np-1) = QN;

    vector<vector<double>> Cinserts;
    for (int k=0; k < C.outerSize(); ++k) {
        vector<double> Cinsert;
        for (SparseMatrix<double>::InnerIterator it(C,k); it; ++it) {
            Cinsert.push_back(it.value()); Cinsert.push_back(it.row()); Cinsert.push_back(it.col()); 
            Cinserts.push_back(Cinsert);
            Cinsert.clear();
        }
    }
    SparseMatrix<double> Cb(Np*C.rows(), Np*C.cols());
    for (int i = 0; i < Np; ++i){
        for (int j = 0; j < Cinserts.size(); ++j) {
            Cb.insert(p*i+Cinserts[j][1], n*i+Cinserts[j][2]) = Cinserts[j][0];
        }
    }

    SparseMatrix<double> Rb(Np,Np);
    for (int i = 0; i < Np; ++i) {Rb.insert(i, i) = R;}

    M1 = 2* ( ( Bb.transpose()* (Cb.transpose()*Qb*Cb) ) *Ab );
    M2 = (-2* (Qb*Cb) *Bb).transpose();

    // Bounds on the states
    MatrixXd Aineq_temp(n*Np*2, Np);
    MatrixXd bineq_temp(n*Np*2, 1);

    Aineq_temp(seq(0, Np*n-1), seq(0, Aineq_temp.cols()-1)) = Bb; Aineq_temp(seq(Np*n, 2*Np*n-1), seq(0, Aineq_temp.cols()-1)) = -Bb;
    bineq_temp(seq(0, Np*n-1), seq(0, bineq_temp.cols()-1)) = Xub - Ab*x0;

    vector<int> indices = NotNanIndex(bineq_temp, bineq_temp.rows());
    MatrixXd Aineq = Aineq_temp(indices, seq(0, Aineq_temp.cols()-1));
    MatrixXd bineq = bineq_temp(indices, seq(0, bineq_temp.cols()-1));

    MatrixXd H = 2*(Bb.transpose()*Cb.transpose()*Qb*Cb*Bb + Rb);
    MatrixXd g = (2*x0.transpose()*Ab.transpose()*Cb.transpose()*Qb*Cb*Bb).transpose() + ulin + Bb.transpose()*(Cb.transpose()*qlin);
    H = (H+H.transpose())/2;
    
    // Initialize controller with qpOASES
    int nV = Np;
    int nC = 2*Np;
    Qp = qpOASES::QProblem(nV, nC);
    qpOASES::int_t nWSR = 1000;
    
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    Qp.setOptions( options );

    convertMatrixtoQpArray(H_qp, H, nV, nV); 
    qpOASES::real_t g_qp[nV]; convertMatrixtoQpArray(g_qp, g, nV, 1);
    qpOASES::real_t Aineq_qp[nC*nV]; convertMatrixtoQpArray(Aineq_qp, Aineq, nC, nV);
    qpOASES::real_t Ulb_qp[nV]; convertMatrixtoQpArray(Ulb_qp, Ulb, nV, 1);
    qpOASES::real_t Uub_qp[nV]; convertMatrixtoQpArray(Uub_qp, Uub, nV, 1);
    qpOASES::real_t bineq_qp[nC]; convertMatrixtoQpArray(bineq_qp, bineq, nC, 1);
    
    qpOASES::SymDenseMat *Hsd = new qpOASES::SymDenseMat(100, 100, 100, H_qp);
    qpOASES::DenseMatrix *Ad = new qpOASES::DenseMatrix(200, 100, 100, Aineq_qp);

    // Solve first QP.
    Qp.init(Hsd, g_qp, Ad, Ulb_qp, Uub_qp, NULL, bineq_qp, nWSR);
}

