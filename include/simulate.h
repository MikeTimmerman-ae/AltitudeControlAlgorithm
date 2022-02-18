/**
 *	\file include/simulate.h
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#pragma once

#include "controller.h"    // #include header
#include "controllerB.h"
#include "dynamics.h"
#include "rbf.h"

using namespace std;

void closed_loop_simulation(controller MyController, dynamics MyDynamics, dynamics MyNomDynamics, rbf liftFun, float Tsim);

void closed_loop_simulation(controllerB MyController, dynamics MyDynamics, dynamics MyNomDynamics, rbf liftFun, float Tsim);
