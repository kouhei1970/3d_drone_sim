#include <stdio.h>
#include "rigidbody.hpp"

void sim(void)
{
    double t = 0.0;
    double h=0.001;
    double T_end = 10.0;
    double force[3];
    double moment[3];

    rigidbody body( 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);

    force[0]=0.0;
    force[1]=0.0;
    force[2]=1.0;
    moment[0]=0.0;
    moment[1]=0.0;
    moment[2]=0.001;

    while (t< T_end)
    {
        body.print(t);
        body.update(force, moment, h);
        t=t+h;
    }
    body.print(t);
}

int main(void)
{
    sim();
    return 0;
}