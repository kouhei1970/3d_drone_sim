#include <stdio.h>
#include "rigidbody.hpp"

void sim(void)
{
    double h=0.001;
    double T_end = 10.0;

    rigidbody body( 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);


}

int main(void)
{
    sim();
    return 0;
}