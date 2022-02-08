/**
 *	\file src/rbf.cpp
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#include <string>              // #include directives        
#include <iostream>

#include "../include/rbf.h"         // #include header

#include <Eigen/Dense>              // #include modules

using namespace std;
using namespace Eigen;

rbf::rbf(MatrixXd &cent, string rbf_typeP, int epsP, int kP) {
    // Initialize the lifting function
    C = cent;
    Nrbf = cent.cols();
    rbf_type = rbf_typeP;
    eps = epsP; k = kP;
}

VectorXd rbf::lift(VectorXd &x) {
    // Create lifted matrix
    int Nstate = x.rows();
    VectorXd Y(Nrbf + Nstate, 1);
    
    // Populate lifted data matrix
    for (int i = Nstate; i < Nrbf + Nstate; ++i) {
        MatrixXd Cstate = C(seq(0, Nstate-1), i-3);
        double r_squared = (x-Cstate).dot(x-Cstate);
        double y;
    
        if (rbf_type == "thinplate") {
            y = r_squared*log(sqrt(r_squared));
        } else if (rbf_type == "gauss") {
            y = exp(- pow(eps, 2) * r_squared);
        } else if (rbf_type == "invquad") {
            y = 1 / (1 + pow(eps, 2) * r_squared);
        } else if (rbf_type == "invmultquad") {
            y = 1 / sqrt(1 + pow(eps,2) * r_squared);
        } else if (rbf_type == "polyharmonic") {
            y = pow(r_squared, k/2) * log(sqrt(r_squared));
        } else {
            cout << "RBF type not recognized";
        } 
        
        if (y == NAN) {
            y = 0;
        }

        Y(i) = y;
    }
    Y(seq(0, Nstate-1)) = x;
    
    return Y;
}
