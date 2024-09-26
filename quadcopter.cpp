#include "quadcopter.hpp"
#include "motor.hpp"
#include "propeller.hpp"
#include "rigidbody.hpp"
//test2

quadcopter::quadcopter()
{
    step = 0.001;
    arm_length = 0.09;
}

void quadcopter::update(double* u)
{
    double force[3]={0.0, 0.0, 0.0};
    double moment[3]={0.0, 0.0, 0.0};

    //motor[0].print(t);
    
    //Motor
    for (int i=0; i<4; i++)
    {
        mot[i].update(u[i], prop[i].torque(mot[i].Omega_mot), step);
    }
    //Force
    force[2]=0.0;
    for (int i=0; i<4; i++)
    {
        force[2] -= prop[i].thrust(mot[i].Omega_mot);
    }
    //Moment
    moment[0] = arm_length*( - prop[0].thrust(mot[0].Omega_mot) 
                    - prop[1].thrust(mot[1].Omega_mot) 
                    + prop[2].thrust(mot[2].Omega_mot) 
                    + prop[3].thrust(mot[3].Omega_mot));
    moment[1] = arm_length*( + prop[0].thrust(mot[0].Omega_mot) 
                    - prop[1].thrust(mot[1].Omega_mot) 
                    - prop[2].thrust(mot[2].Omega_mot) 
                    + prop[3].thrust(mot[3].Omega_mot));
    moment[2] =   mot[0].torque(u[0]) 
                - mot[1].torque(u[1]) 
                + mot[2].torque(u[2]) 
                - mot[3].torque(u[3]);
    //Rigidbody
    body.update(force, moment, step);
}

void quadcopter::print_motor(double t)
{
    mot[0].print(t);
}

void quadcopter::print_body(double t)
{
    body.print(t);
}
