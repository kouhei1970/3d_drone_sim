#include "motor.hpp"

motor::motor(void)
{

}

//Motor equation
void motor::dxdt(double* output, double omega, double dist, double u)
{
    double omega_dot = -(D_mot + K_mot*K_mot/R_mot)*omega - dist + K_mot * u /R_mot;
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

    dxdy(&k1, state, u, dist);
    state2 = state + 0.5*h*k1;
    dxdy(&k2, state2, u, dist);
    state2 = state + 0.5*h*k2;
    dxdy(&k3, state2, u, dist);
    state2 = state + h*k3;
    dxdy(&k4, state2, u, dist);

    state= state+ h*(k1 + 2*k2 + 2*k3 + k4)/6;

    Omega_mot = state;
}
