/**
 *	\file include/helpers.h
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#pragma once

#include <string>                 // #include directives
#include <vector>

#include <Eigen/Dense>            // #include module
#include <qpOASES.hpp>

using namespace Eigen;            // using namespace
using namespace std;

MatrixXd loadFile(string FileName, int row, int col);

void saveToFile(MatrixXd &data, int rows, int cols, string FileName);

vector<int> NotNanIndex(MatrixXd &A, int n);

void convertMatrixtoQpArray(qpOASES::real_t *qpArray, MatrixXd &Matrix, int m, int n);
