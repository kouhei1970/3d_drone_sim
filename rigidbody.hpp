#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#define STATE_NUM 13

void quat2dcm(double* dcm, double* quat);

class rigidbody
{
    private:
        //Body parameters
        double Mass;
        double Ixx, Ixy, Ixz;
        double Iyx, Iyy, Iyz;
        double Izx, Izy, Izz;

        //Velocity
        double Ub, Vb, Wb;
        double Pb, Qb, Rb;

        //Angle
        double Q0, Q1, Q2, Q3;
        double Phi, Theta, Psi;

        //Position
        double Xe, Ye, Ze;

        //Derivative
        void dxdy(double* output, double* state, double* force, double* moment);
        
    public:
        rigidbody();
        rigidbody(  double u, double v, double w,
                    double p, double q, double r,
                    double x, double y, double z,
                    double phi, double theta, double psi);

        //State update (rk4)
        void update(double* force, double* moment, double h);

        //Stare print
        void print(double t);

};

#endif