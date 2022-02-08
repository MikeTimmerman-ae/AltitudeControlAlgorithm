/**
 *	\file header.h
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#pragma once

#include <iostream>                 // #include directives
#include <fstream>
#include <vector>
#include <math.h>
#include <string>

#include <qpOASES.hpp>              // #include modules
#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include "include/controller.h"     // #include src code
#include "include/dynamics.h"

#define PI 3.14159265               // #define constants

using namespace std;                // using namespaces
using namespace Eigen;
using namespace qpOASES;
using namespace N;