/**
 *	\file src/controller.cpp
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#include "../include/controller.h"  // #include header

#include <iostream>                 // #include directives
#include <fstream>
#include <vector>
#include <math.h>
#include <string>

#include <qpOASES.hpp>                       // #include modules
#include <Eigen/Dense>
#include <Eigen/SparseCore>

using namespace N;
using namespace std;
using namespace Eigen;

void my_class::do_something()
{   
    MatrixXd MyMatrix(3,3);
    MyMatrix << 3,6,8,3,7,9,4,7,9;
    cout << MyMatrix << endl;
}
