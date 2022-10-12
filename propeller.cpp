#include "propeller.hpp"

propeller::propeller()
{
    Cq = 3.8e-8;
    Ct = 8.3e-7;
    J_prop = 0.0;
}

double propeller::torque(double omega)
{
    return Cq*omega*omega;
}

double propeller::thrust(double omega)
{
    return Ct*omega*omega;
}