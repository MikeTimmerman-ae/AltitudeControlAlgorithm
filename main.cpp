#include "header.h"



int main(int argc, char const *argv[])
{
    /* Initiate Rocket Dynamics */
    dynamics MyRocket;
    // rbf liftFun();

    for (int i = 0; i < 610; ++i) {
        MyRocket.update_state(0);
    }

    return 0;
}
