#include "header.h"


int main(int argc, char const *argv[])
{   
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

    // float Xub = 8000;
    // float Xlb = 0;

    float Ulb = 0; 
    float Uub = 1; 

    float ulin = 0;
    float qlin = 0;

    controllerB MyController(A, B, C, d, Q, R, Q, Npred, Ulb, Uub, ulin, qlin);

    
    /* Closed-loop simulation */
    // Initiate Actual 1D Rocket Dynamics
    dynamics MyRocket;
    dynamics MyNominalRocket;

    closed_loop_simulation(MyController, MyRocket, MyNominalRocket, liftFun, 34);

    return 0;
}