#include "motor.hpp"
#include <stdio.h>

motor::motor()
{
    J_mot = 8.12e-6;
    D_mot = 0.0;
    K_mot = 3.28e-3;
    R_mot = 0.12;
    J_load = 0.0;
    Omega_mot = 0.0;
}

//Motor equation
void motor::dxdt(double* output, double omega, double dist, double u)
{
    double omega_dot = (-(D_mot + K_mot*K_mot/R_mot)*omega - dist + K_mot * u /R_mot)/(J_mot+J_load);
    //printf("omata_dot:%f", omega_dot);
    *output = omega_dot;
}

//State update (rk4)
void motor::update(double u, double dist, double h)
{
    double k1, k2, k3, k4;
    double state;
    double state2;

    //State
    state = Omega_mot;
    //printf("state:%f %f %f\n", Omega_mot, dist, u);
    dxdt(&k1, state, dist, u);
    state2 = state + 0.5*h*k1;
    dxdt(&k2, state2, dist, u);
    state2 = state + 0.5*h*k2;
    dxdt(&k3, state2, dist, u);
    state2 = state + h*k3;
    dxdt(&k4, state2, dist, u);
    //printf("k:%f %f %f %f\n", k1, k2, k3, k4);

    state= state+ h*(k1 + 2*k2 + 2*k3 + k4)/6;

    Omega_mot = state;
}

void motor::set_load_inartia(double j_load)
{
     J_load = j_load;
}

double motor::torque(double e)
{
    return K_mot*(e - K_mot*Omega_mot);
}

void motor::print(double t)
{
    printf( "%9.6f "
            "%9.6f\n",
        t, 
        Omega_mot);
}
