#include <stdio.h>
#include "rigidbody.hpp"
#include "motor.hpp"
#include "propeller.hpp"

void sim(void)
{
    double t = 0.0;
    double h=0.001;
    double T_end = 1.5;
    double L = 0.09;
    double force[3];
    double moment[3];
    double u;

    rigidbody body( 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
    motor motor[4];
    propeller prop[4];

    force[0]=0.0;
    force[1]=0.0;
    force[2]=0.0;
    moment[0]=0.0;
    moment[1]=0.0;
    moment[2]=0.0;

    while (t< T_end)
    {
        u = 11.1;
        body.print(t);
        //motor[0].print(t);
        
        //Motor
        for (int i=0; i<4; i++)
        {
            motor[i].update(u, prop[i].torque(motor[i].Omega_mot), h);
        }
        //Force
        force[2]=0.0;
        for (int i=0; i<4; i++)
        {
            force[2] -= prop[i].thrust(motor[i].Omega_mot);
        }
        //Moment
        moment[0] = L*( - prop[0].thrust(motor[0].Omega_mot) 
                        - prop[1].thrust(motor[1].Omega_mot) 
                        + prop[2].thrust(motor[2].Omega_mot) 
                        + prop[3].thrust(motor[3].Omega_mot));
        moment[1] = L*( + prop[0].thrust(motor[0].Omega_mot) 
                        - prop[1].thrust(motor[1].Omega_mot) 
                        - prop[2].thrust(motor[2].Omega_mot) 
                        + prop[3].thrust(motor[3].Omega_mot));
        moment[2] =   motor[0].torque(u) 
                    - motor[1].torque(u) 
                    + motor[2].torque(u) 
                    - motor[3].torque(u);
        //Rigidbody
        body.update(force, moment, h);
        t=t+h;
    }
    //motor[0].print(t);
    body.print(t);
}

int main(void)
{
    sim();
    return 0;
}