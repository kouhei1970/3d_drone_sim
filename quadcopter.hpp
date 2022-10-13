#ifndef QUADCOPTER_HPP
#define QUADCOPTER_HPP
#include "motor.hpp"
#include "propeller.hpp"
#include "rigidbody.hpp"

class quadcopter
{
    private:
        motor mot[4];
        propeller prop[4];
        rigidbody body;
        double arm_length;
    public:
        quadcopter();
        double step;
        void update(double* u);
        void print_motor(double t);
        void print_body(double t);

};



#endif