#include <stdio.h>
#include "quadcopter.hpp"

void sim(void)
{
    double t = 0.0;
    double T_end = 1.5;
    double u[4]={11.1, 11.1, 11.1, 11.1};
    quadcopter copter;

    //Simulation
    while (t< T_end)
    {
        copter.print_body(t);
        copter.update(u);
        t = t + copter.step;
    }
    copter.print_body(t);
}

int main(void)
{
    sim();
    return 0;
}