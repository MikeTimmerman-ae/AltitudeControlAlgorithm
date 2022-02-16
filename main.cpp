#include "header.h"

#include "windows.h"
#include "psapi.h"


int main(int argc, char const *argv[])
{
    /* Initiate Actualt 1D Rocket Dynamics */
    dynamics MyRocket;
    dynamics MyNominalRocket;

    /* Initialize lifting mapping function */
    MatrixXf cent = loadFile("../data/cent.csv", P::n_states, P::n_rbf);
    string rbf_type = "thinplate";
    rbf liftFun(cent, rbf_type);

    /* Configure Controller*/
    MatrixXf A = loadFile("../data/Alift.csv", P::n_states_lift, P::n_states_lift);              // x_dot = A*x + B*u
    MatrixXf B = loadFile("../data/Blift.csv", P::n_states_lift, P::n_control_input);
    SparseMatrix<float> C(P::n_output, P::n_states_lift); C.insert(0, 0) = 1;                             // y = C*x

    float d = 0.0;
    float Q = 1.0;
    float R = 0.01;

    int Npred = 100;

    float Xub = 8000;

    float Ulb = 0; 
    float Uub = 1; 

    float ulin = 0;
    float qlin = 0;

    controller MyController(A, B, C, d, Q, R, Q, Npred, Xub, Ulb, Uub, ulin, qlin);

    /* Closed-loop simulation */
    int Nsim = 1700;

    MatrixXf yrr = loadFile("../data/OptimalTrajectory.csv", 3, Nsim);
    MyRocket.state << 2003.754, 599.539;                                          // Initial state
    MyNominalRocket.state << 2003.754, 599.539;                                          
    MyRocket.t = MyRocket.t_burn;                                                 // Initial time
    MyNominalRocket.t = MyNominalRocket.t_burn;
    VectorXf xlift;
    
    MatrixXf X(Nsim+1, P::n_states); X(0, seq(0, P::n_states-1)) = MyRocket.state.transpose();
    MatrixXf Xnom(Nsim+1, P::n_states); Xnom(0, seq(0, P::n_states-1)) = MyNominalRocket.state.transpose();
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
        xlift = liftFun.lift(MyRocket.state);
        float u = MyController.getControlInput(xlift, yr);
        MyRocket.update_state(u);
        qpOASES::real_t toc = qpOASES::getCPUtime();
        MyNominalRocket.update_state(0);
        
        GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));

        // Store data
        X(i+1, seq(0, P::n_states-1)) = MyRocket.state.transpose();
        Xnom(i+1, seq(0, P::n_states-1)) = MyNominalRocket.state.transpose();
        U(i, 0) = u;
        ref(i, 0) = yr;
        CPUtime(i, 0) = toc-tic;
        memoryUse(i, 0) = pmc.PrivateUsage;

        if ((i+1)%25 == 0) {
            cout << "Altitude: " << MyRocket.state[0] << " Reference point: " << yr << " Nominal: " << MyNominalRocket.state[0] << " Time: " << MyRocket.t << " Control input: " << u << endl;
            cout << "Closed-Loop simulation: iteration " << i+1 << " out of " << Nsim << endl;
        }
    }

    // Store data in csv files
    saveToFile(ref, ref.rows(), ref.cols(), "../data/reference.csv");
    saveToFile(X, X.rows(), X.cols(), "../data/state.csv");
    saveToFile(U, U.rows(), U.cols(), "../data/control.csv");
    saveToFile(Xnom, Xnom.rows(), Xnom.cols(), "../data/nomState.csv");
    saveToFile(CPUtime, CPUtime.rows(), CPUtime.cols(), "../data/cpuTime.csv");
    saveToFile(memoryUse, memoryUse.rows(), memoryUse.cols(), "../data/memoryUse.csv");

    return 0;
}