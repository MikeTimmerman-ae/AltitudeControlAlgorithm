/**
 *	\file include/rbf.h
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#pragma once

#include <string>               // #include directives        

#include <Eigen/Dense>          // #include module

using namespace Eigen;          // using namespace of module
using namespace std;

class rbf
{   
    private:
        // Private class members
        MatrixXf C;
        string rbf_type;
        int eps, k, Nrbf;
    public:
        // Constructor
        rbf(MatrixXf &cent, string rbf_typeP, int epsP = 1, int kP = 1);
        // Class methods
        VectorXf lift(VectorXf &x);
};