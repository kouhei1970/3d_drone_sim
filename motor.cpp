#include "motor.hpp"

motor::motor(void)
{

}

//Motor equation
void motor::dxdt(double* output, double* state, double dist, double u)
{
    double omega = state[0];
    double omega_dot = -(D_mot + K_mot*K_mot/R_mot)*omega - dist + K_mot * u /R_mot;
    output[0] = omega_dot;
}