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
        VectorXf k1;
        VectorXf k2;
        VectorXf k3;
        VectorXf k4;
        VectorXf state_derivative;
        VectorXf rocket_dynamics(float t, VectorXf stateEv, float u);
    public:
        // Public class members
        float dt;
        float t;
        int n_states;
        float g;
        float density_sea;
        float A;
        float A_flap;
        float Cd;
        float mass;
        float t_burn;
        float thrust;
        VectorXf state;
        
        // Constructor
        dynamics();
        dynamics(int n, VectorXf initState, float dtP, float t_initial);
        
        // Class methods
        void update_state(float u);
};

