#include <stdio.h>
#include "quadcopter.hpp"
#include "rigidbody.hpp"
#include "motor.hpp"
#include "propeller.hpp"

void sim(void)
{
    double t = 0.0;
    double T_end = 1.5;
    double u[4]={11.1, 11.1, 11.1, 11.1};
    quadcopter copter;
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