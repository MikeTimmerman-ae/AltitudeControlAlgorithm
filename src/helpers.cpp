/**
 *	\file src/helpers.cpp
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#include "../include/parameters.h"  // #include header
#include "../include/helpers.h"     // #include header

#include <vector>                   // #include directives
#include <string>
#include <fstream>
#include <iostream>

#include <qpOASES.hpp>              // #include modules
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

MatrixXf loadFile(string FileName, int row, int col) {
    MatrixXf Matrix(row, col);
	ifstream File(FileName);
	for (int k = 0; k < row; ++k) {
		string line;
		getline(File, line, '\n');
		for (int j = 0; j < col; ++j) {
			string entry = line.substr(0, line.find(','));
			line.erase(0, line.find(',') + 1);
			Matrix(k, j) = stod(entry);
		}
	}
    return Matrix;
}

void saveToFile(MatrixXf &data, int rows, int cols, string FileName) {
    ofstream File; File.open(FileName);
    for (int k = 0; k < rows; ++k) {
        string line = "";
        for (int j = 0; j < cols; ++j) {
            line.append(to_string(data(k, j)));
            line.append(",");
        }
        line.pop_back();
        File << line << "\n";
    }
    File.close();
}

vector<int> NotNanIndex(MatrixXf &A, int n) {
    vector<int> indices;
    for (int i = 0; i < n; ++i) {
        if (!isnan(A(i, 0)))
            indices.push_back(i);
    }
    return indices;
}

void convertMatrixtoQpArray(qpOASES::real_t *qpArray, MatrixXf &Matrix, int m, int n) {
    int k = 0;
    for (int i = 0; i < m; ++i) {
        for (int j=0; j < n; ++j) {
            *(qpArray + k) = Matrix(i, j);
            ++k;
        }
    }
}
