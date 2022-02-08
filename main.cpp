#include "header.h"



int main(int argc, char const *argv[])
{
    /* Initiate Rocket Dynamics */
    dynamics MyRocket;


    for (int i = 0; i < 4700; ++i) {
        MyRocket.update_state(0);
        cout << MyRocket.state[0] << endl;
    }

    return 0;
}
