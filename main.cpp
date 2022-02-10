#include "header.h"



int main(int argc, char const *argv[])
{
    /* Initiate Actualt 1D Rocket Dynamics */
    dynamics MyRocket;

    /* Initialize lifting mapping function */
    MatrixXd cent = loadFile("../data/cent.csv", P::n_states, P::n_rbf);
    string rbf_type = "thinplate";
    rbf liftFun(cent, rbf_type);

    /* Configure Controller*/
    MatrixXd A = loadFile("../data/Alift.csv", P::n_states_lift, P::n_states_lift);              // x_dot = A*x + B*u
    MatrixXd B = loadFile("../data/Blift.csv", P::n_states_lift, P::n_control_input);
    SparseMatrix<double> C(P::n_output, P::n_states_lift); C.insert(0, 0) = 1;                             // y = C*x

    double d = 0.0;
    double Q = 1.0;
    double R = 0.01;

    int Npred = 100;

    double Xub = 3000;

    double Ulb = 0; 
    double Uub = 1; 

    double ulin = 0;
    double qlin = 0;

    controller MyController(A, B, C, d, Q, R, Q, Npred, Xub, Ulb, Uub, ulin, qlin);
    // rbf liftFun();

    for (int i = 0; i < 610; ++i) {
        MyRocket.update_state(0);
    }

    return 0;
}
