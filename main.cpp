#include "header.h"



int main(int argc, char const *argv[])
{
    /* Initiate Rocket Dynamics */
    dynamics MyRocket;

    MatrixXd A(P::n_states, P::n_states);
    MatrixXd B(P::n_states, P::n_control_input);
    SparseMatrix<double> C(1, P::n_states);
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
