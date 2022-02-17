/**
 *	\file include/controllerB.h
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

class controllerB
{
    private:
        // Private class members
        int Np, p, n, m;
        float Q, d;
        MatrixXf Ab;
        MatrixXf Xub;
        MatrixXf Ulb, Uub;
        VectorXf ulin;
        MatrixXf M1, M2;
        SparseMatrix<float> C;
        qpOASES::QProblemB Qp; 
        qpOASES::real_t H_qp[100*100];
        
        // Private class method
        void initialize(MatrixXf &A, MatrixXf &B, MatrixXf &qlin, float R, float QN);
    public:
        // Constructors
        controllerB(MatrixXf &A, MatrixXf &B, SparseMatrix<float> &Cpar, float d_par, float Q_par, float R, float QN,
            Matrix<float, P::n_control_input * P::n_prediction, 1> &Ulb_par, Matrix<float, P::n_control_input * P::n_prediction, 1> &Uub_par,
            Matrix<float, P::n_control_input * P::n_prediction, 1> &ulin_par, Matrix<float, P::n_output * P::n_prediction , 1> &qlin);
        
        controllerB(MatrixXf &A, MatrixXf &B, SparseMatrix<float> &Cpar, float d_par, float Q_par, float R, float QN, int Npred,
            Matrix<float, P::n_control_input, 1> &Ulb_par, Matrix<float, P::n_control_input, 1> &Uub_par,
            Matrix<float, P::n_control_input, 1> &ulin_par, Matrix<float, P::n_output, 1> &qlin);
        
        controllerB(MatrixXf &A, MatrixXf &B, SparseMatrix<float> &Cpar, float d_par, float Q_par, float R, float QN, int Npred,
            float Ulb_par, float Uub_par, float ulin_par, float qlin_par);
        
        // Public class method
        float getControlInput(VectorXf &x0, float yrr);
};
