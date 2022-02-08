/**
 *	\file src/dynamics.cpp
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#include <iostream>
#include "../include/dynamics.h"    // #include header

#include <Eigen/Dense>              // #include modules

using namespace std;
using namespace Eigen;

dynamics::dynamics() {
    // Initialize system dynamics properties
    dt = 0.01;
    t = 0;
    n_states = 2;
    state = VectorXd::Zero(2,1);

    // Standard rocket parameters
    g = 9.81;
    density_sea = 1.225;
    A = 0.0103;
    A_flap = 0.0515;
    Cd = 0.54;
    mass = 19.308;
    t_burn = 6.09;
    thrust = 2501.8;

    // Initialize private data members
    k1 = VectorXd::Zero(2,1);
    k2 = VectorXd::Zero(2,1);
    k3 = VectorXd::Zero(2,1);
    k4 = VectorXd::Zero(2,1);
    state_derivative = VectorXd::Zero(2,1);
}

dynamics::dynamics(int n, VectorXd initState, double dtP, double t_initial) {
    // Initialize system dynamics properties
    dt = dtP;
    t = t_initial;
    n_states = n;
    state = initState;

    // Standard rocket parameters
    g = 9.81;
    density_sea = 1.225;
    A = 0.0103;
    A_flap = 0.0515;
    Cd = 0.54;
    mass = 19.308;
    t_burn = 6.09;
    thrust = 2501.8;

    // Initialize private data members
    k1 = VectorXd::Zero(n,1);
    k2 = VectorXd::Zero(n,1);
    k3 = VectorXd::Zero(n,1);
    k4 = VectorXd::Zero(n,1);
    state_derivative = VectorXd::Zero(n,1);
}

VectorXd dynamics::rocket_dynamics(double tEv, VectorXd stateEv, double u) {
    /*
    t - double representing the independent variable of the DE
    y - 1-D array representing the state variable of the DE
    u - double representing the control input of the DE
    */
    double current_thrust = (tEv < t_burn) ? thrust : 0;
    double density = density_sea * exp(-stateEv[0] / 8800.0);

    state_derivative[0] = stateEv[1];
    state_derivative[1] = -g - 1.0/2.0 * density * pow(stateEv[1], 2) * (A + u*A_flap) * Cd / mass + current_thrust / mass;

    return state_derivative;
}

void dynamics::update_state(double u) {
    // Evaluation at start of interval
    k1 = rocket_dynamics(t, state, u);

    // Evaluation at midway of interval
    k2 = rocket_dynamics(t + 1/2*dt, state + dt*k1/2.0, u);

    k3 = rocket_dynamics(t + 1/2*dt, state + dt*k2/2.0, u);

    // Evaluation at end of interval
    k4 = rocket_dynamics(t + dt, state + dt*k3, u);

    // Update system dynamics properties   
    state = state + dt*(k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
    t = t + dt;
}

