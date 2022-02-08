/**
 *	\file include/dynamics.h
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class dynamics
{
    private:
        VectorXd k1;
        VectorXd k2;
        VectorXd k3;
        VectorXd k4;
        VectorXd state_derivative;
        VectorXd rocket_dynamics(double t, VectorXd stateEv, double u);
    public:
        // Public class members
        double dt;
        double t;
        int n_states;
        double g;
        double density_sea;
        double A;
        double A_flap;
        double Cd;
        double mass;
        double t_burn;
        double thrust;
        VectorXd state;
        
        // Constructor
        dynamics();
        dynamics(int n, VectorXd initState, double dtP, double t_initial);
        
        // Class methods
        void update_state(double u);
};

