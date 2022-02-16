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

MatrixXf loadFile(string FileName, int row, int col);

void saveToFile(MatrixXf &data, int rows, int cols, string FileName);

vector<int> NotNanIndex(MatrixXf &A, int n);

void convertMatrixtoQpArray(qpOASES::real_t *qpArray, MatrixXf &Matrix, int m, int n);
