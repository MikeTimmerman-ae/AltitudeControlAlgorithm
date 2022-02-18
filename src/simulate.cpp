/**
 *	\file src/simulate.cpp
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#include <iostream>                  // #include directives
#include "windows.h"
#include "psapi.h"

#include "../include/simulate.h"    // #include header
#include "../include/controller.h"
#include "../include/controllerB.h"
#include "../include/dynamics.h"
#include "../include/rbf.h"
#include "../include/helpers.h"

#include "../include/parameters.h"  // #include parameters

#include <Eigen/Dense>              // #include module
#include <qpOASES.hpp>

void closed_loop_simulation(controller MyController, dynamics MyDynamics, dynamics MyNomDynamics, rbf liftFun, float Tsim) {
    int hi;
}


void closed_loop_simulation(controllerB MyController, dynamics MyDynamics, dynamics MyNomDynamics, rbf liftFun, float Tsim) {
    /* Closed-loop simulation */
    int Nsim = (int) Tsim/MyDynamics.dt;

    MatrixXf yrr = loadFile("../data/OptimalTrajectory.csv", 3, Nsim);
    MyDynamics.state << 2003.754, 599.539;                                          // Initial state
    MyNomDynamics.state << 2003.754, 599.539;                                          
    MyDynamics.t = MyDynamics.t_burn;                                               // Initial time
    MyNomDynamics.t = MyNomDynamics.t_burn;
    VectorXf xlift;
    
    MatrixXf X(Nsim+1, P::n_states); X(0, seq(0, P::n_states-1)) = MyDynamics.state.transpose();
    MatrixXf Xnom(Nsim+1, P::n_states); Xnom(0, seq(0, P::n_states-1)) = MyNomDynamics.state.transpose();
    MatrixXf U(Nsim, 1);
    MatrixXf ref(Nsim, 1);
    MatrixXf CPUtime(Nsim, 1);
    MatrixXf memoryUse(Nsim, 1);

    PROCESS_MEMORY_COUNTERS_EX pmc;
    
    for (int i = 0; i < Nsim; ++i) {
        qpOASES::real_t tic = qpOASES::getCPUtime();
        // Current value of the reference signal
        float yr = yrr(1, i);

        // Simulate closed loop feedback control
        VectorXf state = MyDynamics.state;
        xlift = liftFun.lift(state);
        float u = MyController.getControlInput(xlift, yr);
        MyDynamics.update_state(u);
        qpOASES::real_t toc = qpOASES::getCPUtime();
        MyNomDynamics.update_state(0);
        
        GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));

        // Store data
        X(i+1, seq(0, P::n_states-1)) = MyDynamics.state.transpose();
        Xnom(i+1, seq(0, P::n_states-1)) = MyNomDynamics.state.transpose();
        U(i, 0) = u;
        ref(i, 0) = yr;
        CPUtime(i, 0) = toc-tic;
        memoryUse(i, 0) = pmc.PrivateUsage;

        if ((i+1)%25 == 0) {
            cout << "Altitude: " << MyDynamics.state[0] << " Reference point: " << yr << " Nominal: " << MyNomDynamics.state[0] << " Time: " << MyDynamics.t << " Control input: " << u << endl;
            cout << "Closed-Loop simulation: iteration " << i+1 << " out of " << Nsim << endl;
        }
    }

    // Store data in csv files
    saveToFile(ref, ref.rows(), ref.cols(), "../data/reference.csv");
    saveToFile(X, X.rows(), X.cols(), "../data/state.csv");
    saveToFile(U, U.rows(), U.cols(), "../data/control.csv");
    saveToFile(Xnom, Xnom.rows(), Xnom.cols(), "../data/nomState.csv");
    saveToFile(CPUtime, CPUtime.rows(), CPUtime.cols(), "../data/cpuTime.csv");
    saveToFile(memoryUse, memoryUse.rows(), memoryUse.cols(), "../data/memoryUse.csv");}