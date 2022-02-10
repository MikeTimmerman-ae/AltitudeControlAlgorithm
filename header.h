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
#include "include/rbf.h"
#include "include/helpers.h"
#include "include/parameters.h"

using namespace std;                // using namespaces
using namespace Eigen;